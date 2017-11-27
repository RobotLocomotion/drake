#include "drake/solvers/test/semidefinite_program_examples.h"

#include <limits>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/solvers/test/mathematical_program_test_util.h"

namespace drake {
namespace solvers {
namespace test {
using Eigen::Vector2d;
using Eigen::Matrix3d;
using Eigen::Vector3d;
using Eigen::Matrix4d;

void TestTrivialSDP(const MathematicalProgramSolverInterface& solver) {
  MathematicalProgram prog;

  auto S = prog.NewSymmetricContinuousVariables<2>("S");

  // S is p.s.d
  prog.AddPositiveSemidefiniteConstraint(S);

  // S(1, 0) = 1
  prog.AddBoundingBoxConstraint(1, 1, S(1, 0));

  // Min S.trace()
  prog.AddLinearCost(S.cast<symbolic::Expression>().trace());

  RunSolver(&prog, solver);

  auto S_value = prog.GetSolution(S);

  // Choose 1E-8 since that is Mosek's default feasibility tolerance.
  EXPECT_TRUE(CompareMatrices(S_value, Eigen::Matrix2d::Ones(), 1E-8));
}

void FindCommonLyapunov(const MathematicalProgramSolverInterface& solver) {
  MathematicalProgram prog;
  auto P = prog.NewSymmetricContinuousVariables<3>("P");
  prog.AddPositiveSemidefiniteConstraint(P - 1E-3 * Matrix3d::Identity());
  Eigen::Matrix3d A1;
  // clang-format off
  A1 << -1, -1, -2,
      0, -1, -3,
      0, 0, -1;
  // clang-format on
  auto binding1 = prog.AddPositiveSemidefiniteConstraint(
      -A1.transpose() * P - P * A1 - 1E-3 * Matrix3d::Identity());

  Eigen::Matrix3d A2;
  // clang-format off
  A2 << -1, -1.2, -1.8,
      0, -0.7, -2,
      0, 0, -0.4;
  // clang-format on
  auto binding2 = prog.AddPositiveSemidefiniteConstraint(
      -A2.transpose() * P - P * A2 - 1E-3 * Matrix3d::Identity());

  RunSolver(&prog, solver);

  const Matrix3d P_value = prog.GetSolution(P);
  const auto Q1_flat_value = prog.GetSolution(binding1.variables());
  const auto Q2_flat_value = prog.GetSolution(binding2.variables());
  const Eigen::Map<const Matrix3d> Q1_value(&Q1_flat_value(0));
  const Eigen::Map<const Matrix3d> Q2_value(&Q2_flat_value(0));
  Eigen::SelfAdjointEigenSolver<Matrix3d> eigen_solver_P(P_value);

  // The comparison tolerance is set as 1E-8, to match the Mosek default
  // feasibility tolerance 1E-8.
  EXPECT_TRUE(CompareMatrices(P_value, P_value.transpose(),
                              std::numeric_limits<double>::epsilon(),
                              MatrixCompareType::absolute));
  EXPECT_GE(eigen_solver_P.eigenvalues().minCoeff(), 0);
  Eigen::SelfAdjointEigenSolver<Matrix3d> eigen_solver_Q1(Q1_value);
  EXPECT_GE(eigen_solver_Q1.eigenvalues().minCoeff(), 0);
  Eigen::SelfAdjointEigenSolver<Matrix3d> eigen_solver_Q2(Q2_value);
  EXPECT_GE(eigen_solver_Q2.eigenvalues().minCoeff(), 0);
  EXPECT_TRUE(CompareMatrices(
      A1.transpose() * P_value + P_value * A1 + 1E-3 * Matrix3d::Identity(),
      -Q1_value, 1E-8, MatrixCompareType::absolute));
  EXPECT_TRUE(CompareMatrices(
      A2.transpose() * P_value + P_value * A2 + 1E-3 * Matrix3d::Identity(),
      -Q2_value, 1E-8, MatrixCompareType::absolute));
}

void FindOuterEllipsoid(const MathematicalProgramSolverInterface& solver) {
  std::array<Matrix3d, 3> Q;
  std::array<Vector3d, 3> b;
  Q[0] = Matrix3d::Identity();
  b[0] = Vector3d::Zero();
  // clang-format off
  Q[1] << 1, 0.2, 0.3,
      0.2, 2,  0.6,
      0.3, 0.6, 3;
  b[1] << 0.3, 2, 1;
  Q[2] << 1, -0.1, 0.2,
      -0.1, 4, 0.3,
      0.2, 0.3, 3;
  b[2] << 2, -1, 3;
  // clang-format on
  MathematicalProgram prog;
  auto P = prog.NewSymmetricContinuousVariables<3>("P");
  prog.AddPositiveSemidefiniteConstraint(P);
  auto s = prog.NewContinuousVariables<3>("s");
  prog.AddBoundingBoxConstraint(0, std::numeric_limits<double>::infinity(), s);
  auto c = prog.NewContinuousVariables<3>("c");

  for (int i = 0; i < 3; ++i) {
    Eigen::Matrix<symbolic::Expression, 4, 4> M{};
    // clang-format off
    M << s(i) * Q[i] - P, s(i) * b[i] - c,
        s(i) * b[i].transpose() - c.transpose(), 1 - s(i);
    // clang-format on
    prog.AddPositiveSemidefiniteConstraint(M);
  }

  // I have to compute trace of P in the loop, instead of using P.trace(),
  // since Variable cannot be cast to Expression implicitly.
  symbolic::Expression P_trace{0};
  for (int i = 0; i < 3; ++i) {
    P_trace += P(i, i);
  }
  prog.AddLinearCost(-P_trace);

  RunSolver(&prog, solver);

  auto P_value = prog.GetSolution(P);
  auto s_value = prog.GetSolution(s);
  auto c_value = prog.GetSolution(c);

  Eigen::SelfAdjointEigenSolver<Matrix3d> es_P(P_value);
  EXPECT_TRUE((es_P.eigenvalues().array() >= -1E-6).all());
  for (int i = 0; i < 3; ++i) {
    Matrix4d M_value;
    // clang-format off
    M_value << s_value(i) * Q[i] - P_value, s_value(i) * b[i] - c_value,
        s_value(i) * b[i].transpose() - c_value.transpose(), 1 - s_value(i);
    // clang-format on
    Eigen::SelfAdjointEigenSolver<Matrix4d> es_M(M_value);
    EXPECT_TRUE((es_M.eigenvalues().array() >= -1E-6).all());
  }
}

void SolveEigenvalueProblem(const MathematicalProgramSolverInterface& solver) {
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<2>("x");
  Matrix3d F1;
  // clang-format off
  F1 << 1, 0.2, 0.3,
      0.2, 2, -0.1,
      0.3, -0.1, 4;
  Matrix3d F2;
  F2 << 2, 0.4, 0.7,
      0.4, -1, 0.1,
      0.7, 0.1, 5;
  // clang-format on
  auto z = prog.NewContinuousVariables<1>("z");
  prog.AddLinearMatrixInequalityConstraint(
      {Matrix3d::Zero(), Matrix3d::Identity(), -F1, -F2}, {z, x});

  Vector2d x_lb(0.1, 1);
  Vector2d x_ub(2, 3);
  prog.AddBoundingBoxConstraint(x_lb, x_ub, x);

  prog.AddLinearCost(drake::Vector1d(1), z);

  RunSolver(&prog, solver);

  const double z_value = prog.GetSolution(z(0));
  auto x_value = prog.GetSolution(x);
  auto xF_sum = x_value(0) * F1 + x_value(1) * F2;

  Eigen::SelfAdjointEigenSolver<Matrix3d> eigen_solver_xF(xF_sum);
  // The comparison tolerance is set to 1E-7, slightly larger than Mosek's
  // default feasibility tolerance 1E-8.
  EXPECT_NEAR(z_value, eigen_solver_xF.eigenvalues().maxCoeff(), 1E-7);
}
}  // namespace test
}  // namespace solvers
}  // namespace drake
