#include "drake/solvers/test/semidefinite_program_examples.h"

#include <algorithm>
#include <array>
#include <limits>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/math/matrix_util.h"
#include "drake/solvers/test/mathematical_program_test_util.h"

namespace drake {
namespace solvers {
namespace test {
using Eigen::Matrix3d;
using Eigen::Matrix4d;
using Eigen::Vector2d;
using Eigen::Vector3d;
using symbolic::Expression;

const double kInf = std::numeric_limits<double>::infinity();
void TestTrivialSDP(const SolverInterface& solver, double tol) {
  MathematicalProgram prog;

  auto S = prog.NewSymmetricContinuousVariables<3>("S");

  // S is p.s.d
  prog.AddPositiveSemidefiniteConstraint(S);

  // S(1, 0) = 1
  prog.AddBoundingBoxConstraint(1, 1, S(1, 0));

  // Min S.trace()
  prog.AddLinearCost(S.cast<symbolic::Expression>().trace());

  const MathematicalProgramResult result = RunSolver(prog, solver);

  auto S_value = result.GetSolution(S);

  Eigen::Matrix3d S_expected = Eigen::Matrix3d::Zero();
  S_expected.topLeftCorner<2, 2>() = Eigen::Matrix2d::Ones();
  EXPECT_TRUE(CompareMatrices(S_value, S_expected, tol));
  EXPECT_NEAR(result.get_optimal_cost(), 2.0, tol);
}

void FindCommonLyapunov(const SolverInterface& solver,
                        const std::optional<SolverOptions>& solver_options,
                        double tol) {
  MathematicalProgram prog;
  auto P = prog.NewSymmetricContinuousVariables<3>("P");
  const double psd_epsilon{1E-3};
  prog.AddLinearMatrixInequalityConstraint(P -
                                           psd_epsilon * Matrix3d::Identity());
  Eigen::Matrix3d A1;
  // clang-format off
  A1 << -1, -1, -2,
         0, -1, -3,
         0, 0, -1;
  // clang-format on
  const Matrix3<symbolic::Expression> Q1 =
      -A1.transpose() * P - P * A1 - psd_epsilon * Matrix3d::Identity();
  auto binding1 = prog.AddLinearMatrixInequalityConstraint(Q1);

  Eigen::Matrix3d A2;
  // clang-format off
  A2 << -1, -1.2, -1.8,
      0, -0.7, -2,
      0, 0, -0.4;
  // clang-format on
  const Matrix3<symbolic::Expression> Q2 =
      -A2.transpose() * P - P * A2 - psd_epsilon * Matrix3d::Identity();
  auto binding2 = prog.AddLinearMatrixInequalityConstraint(Q2);

  const MathematicalProgramResult result =
      RunSolver(prog, solver, {}, solver_options);

  ASSERT_TRUE(result.is_success());

  const Matrix3d P_value = result.GetSolution(P);

  Eigen::Matrix3d Q1_value = binding1.evaluator()->F()[0];
  for (int i = 0; i < binding1.variables().rows(); ++i) {
    Q1_value += binding1.evaluator()->F()[1 + i] *
                result.GetSolution(binding1.variables()(i));
  }
  Eigen::Matrix3d Q2_value = binding2.evaluator()->F()[0];
  for (int i = 0; i < binding2.variables().rows(); ++i) {
    Q2_value += binding2.evaluator()->F()[1 + i] *
                result.GetSolution(binding2.variables()(i));
  }
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
  EXPECT_TRUE(CompareMatrices(A1.transpose() * P_value + P_value * A1 +
                                  psd_epsilon * Matrix3d::Identity(),
                              -Q1_value, tol, MatrixCompareType::absolute));
  EXPECT_TRUE(CompareMatrices(A2.transpose() * P_value + P_value * A2 +
                                  psd_epsilon * Matrix3d::Identity(),
                              -Q2_value, tol, MatrixCompareType::absolute));
}

void FindOuterEllipsoid(const SolverInterface& solver,
                        const std::optional<SolverOptions>& solver_options,
                        double tol) {
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
  prog.AddBoundingBoxConstraint(0, kInf, s);
  auto c = prog.NewContinuousVariables<3>("c");

  for (int i = 0; i < 3; ++i) {
    Eigen::Matrix<symbolic::Expression, 4, 4> M{};
    // clang-format off
    M << s(i) * Q[i] - P, s(i) * b[i] - c,
        s(i) * b[i].transpose() - c.transpose(), 1 - s(i);
    // clang-format on
    prog.AddLinearMatrixInequalityConstraint(M);
  }

  prog.AddLinearCost(-P.cast<symbolic::Expression>().trace());

  const MathematicalProgramResult result =
      RunSolver(prog, solver, {}, solver_options);

  const auto P_value = result.GetSolution(P);
  const auto s_value = result.GetSolution(s);
  const auto c_value = result.GetSolution(c);

  EXPECT_NEAR(-P_value.trace(), result.get_optimal_cost(), tol);

  const Eigen::SelfAdjointEigenSolver<Matrix3d> es_P(P_value);
  EXPECT_TRUE((es_P.eigenvalues().array() >= -tol).all());
  // The minimal eigen value of M should be 0, since the optimality happens at
  // the boundary of the PSD cone.
  double M_min_eigenvalue = kInf;
  for (int i = 0; i < 3; ++i) {
    Matrix4d M_value;
    // clang-format off
    M_value << s_value(i) * Q[i] - P_value, s_value(i) * b[i] - c_value,
        s_value(i) * b[i].transpose() - c_value.transpose(), 1 - s_value(i);
    // clang-format on
    Eigen::SelfAdjointEigenSolver<Matrix4d> es_M(M_value);
    EXPECT_TRUE((es_M.eigenvalues().array() >= -tol).all());
    M_min_eigenvalue =
        std::min(M_min_eigenvalue, es_M.eigenvalues().minCoeff());
  }
  EXPECT_NEAR(M_min_eigenvalue, 0, tol);
}

void SolveEigenvalueProblem(const SolverInterface& solver,
                            const std::optional<SolverOptions>& solver_options,
                            double tol, bool check_dual) {
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
  auto lmi_constraint = prog.AddLinearMatrixInequalityConstraint(
      {Matrix3d::Zero(), Matrix3d::Identity(), -F1, -F2}, {z, x});

  const Vector2d x_lb(0.1, 1);
  const Vector2d x_ub(2, 3);
  prog.AddBoundingBoxConstraint(x_lb, x_ub, x);

  prog.AddLinearCost(z(0));

  const MathematicalProgramResult result =
      RunSolver(prog, solver, {}, solver_options);

  const double z_value = result.GetSolution(z(0));
  const auto x_value = result.GetSolution(x);
  const auto xF_sum = x_value(0) * F1 + x_value(1) * F2;

  EXPECT_NEAR(z_value, result.get_optimal_cost(), tol);
  Eigen::SelfAdjointEigenSolver<Matrix3d> eigen_solver_xF(xF_sum);
  EXPECT_NEAR(z_value, eigen_solver_xF.eigenvalues().maxCoeff(), tol);
  EXPECT_TRUE(((x_value - x_lb).array() >= -tol).all());
  EXPECT_TRUE(((x_value - x_ub).array() <= tol).all());
  if (check_dual) {
    const Eigen::MatrixXd lmi_dual =
        math::ToSymmetricMatrixFromLowerTriangularColumns(
            result.GetDualSolution(lmi_constraint));
    Eigen::MatrixXd lmi_sol = lmi_constraint.evaluator()->F()[0];
    for (int i = 0; i < ssize(lmi_constraint.evaluator()->F()) - 1; ++i) {
      lmi_sol += lmi_constraint.evaluator()->F()[1 + i] *
                 result.GetSolution(lmi_constraint.variables()(i));
    }
    // Check complementary slackness.
    EXPECT_NEAR((lmi_sol * lmi_dual).trace(), 0, tol);
    // Check if the lmi dual is psd.
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> es_lmi_dual(lmi_dual);
    EXPECT_TRUE((es_lmi_dual.eigenvalues().array() >= -tol).all());
  }
}

void SolveSDPwithSecondOrderConeExample1(const SolverInterface& solver,
                                         double tol) {
  MathematicalProgram prog;
  auto X = prog.NewSymmetricContinuousVariables<3>();
  auto x = prog.NewContinuousVariables<3>();
  Eigen::Matrix3d C0;
  // clang-format off
  C0 << 2, 1, 0,
        1, 2, 1,
        0, 1, 2;
  // clang-format on
  prog.AddLinearCost((C0 * X.cast<symbolic::Expression>()).trace() + x(0));
  prog.AddLinearConstraint(
      (Matrix3d::Identity() * X.cast<Expression>()).trace() + x(0) == 1);
  prog.AddLinearConstraint(
      (Matrix3d::Ones() * X.cast<Expression>()).trace() + x(1) + x(2) == 0.5);
  prog.AddPositiveSemidefiniteConstraint(X);
  prog.AddLorentzConeConstraint(x.cast<symbolic::Expression>());

  MathematicalProgramResult result;
  solver.Solve(prog, {}, {}, &result);
  EXPECT_TRUE(result.is_success());

  const auto X_val = result.GetSolution(X);
  const auto x_val = result.GetSolution(x);
  EXPECT_NEAR((C0 * X_val).trace() + x_val(0), result.get_optimal_cost(), tol);
  EXPECT_NEAR((Eigen::Matrix3d::Identity() * X_val).trace() + x_val(0), 1, tol);
  EXPECT_NEAR((Eigen::Matrix3d::Ones() * X_val).trace() + x_val(1) + x_val(2),
              0.5, tol);
  EXPECT_GE(x_val(0),
            std::sqrt(x_val(1) * x_val(1) + x_val(2) * x_val(2)) - tol);
}

void SolveSDPwithSecondOrderConeExample2(const SolverInterface& solver,
                                         double tol) {
  MathematicalProgram prog;
  const auto X = prog.NewSymmetricContinuousVariables<3>();
  const auto x = prog.NewContinuousVariables<1>()(0);
  prog.AddLinearCost(X(0, 0) + X(1, 1) + x);
  prog.AddBoundingBoxConstraint(0, kInf, x);
  prog.AddLinearConstraint(X(0, 0) + 2 * X(1, 1) + X(2, 2) + 3 * x == 3);
  Vector3<symbolic::Expression> lorentz_cone_expr;
  lorentz_cone_expr << X(0, 0), X(1, 1) + x, X(1, 1) + X(2, 2);
  prog.AddLorentzConeConstraint(lorentz_cone_expr);
  prog.AddLinearConstraint(X(1, 0) + X(2, 1) == 1);
  prog.AddPositiveSemidefiniteConstraint(X);

  MathematicalProgramResult result;
  solver.Solve(prog, {}, {}, &result);
  EXPECT_TRUE(result.is_success());
  const auto X_val = result.GetSolution(X);
  const auto x_val = result.GetSolution(x);
  EXPECT_NEAR(result.get_optimal_cost(), X_val(0, 0) + X_val(1, 1) + x_val,
              tol);
  Eigen::SelfAdjointEigenSolver<Matrix3d> es(X_val);
  EXPECT_TRUE((es.eigenvalues().array() >= -tol).all());
  EXPECT_NEAR(X_val(0, 0) + 2 * X_val(1, 1) + X_val(2, 2) + 3 * x_val, 3, tol);
  EXPECT_GE(X_val(0, 0), std::sqrt(std::pow(X_val(1, 1) + x_val, 2) +
                                   std::pow(X_val(1, 1) + X_val(2, 2), 2)) -
                             tol);
  EXPECT_NEAR(X_val(1, 0) + X_val(2, 1), 1, tol);
  EXPECT_GE(x_val, -tol);
}

void SolveSDPwithOverlappingVariables(const SolverInterface& solver,
                                      double tol) {
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<6>();
  Matrix3<symbolic::Variable> psd_mat1;
  Matrix3<symbolic::Variable> psd_mat2;
  // clang-format off
  psd_mat1 << x(0), x(1), x(3),
              x(1), x(0), x(4),
              x(3), x(4), x(5);
  psd_mat2 << x(0), x(2), x(3),
              x(2), x(0), x(4),
              x(3), x(4), x(5);
  // clang-format on
  prog.AddPositiveSemidefiniteConstraint(psd_mat1);
  prog.AddPositiveSemidefiniteConstraint(psd_mat2);
  prog.AddBoundingBoxConstraint(1, 1, x(1));
  prog.AddLinearCost(2 * x(0) + x(2) + x(5));

  MathematicalProgramResult result;
  solver.Solve(prog, {}, {}, &result);
  EXPECT_TRUE(result.is_success());
  Vector6<double> x_expected;
  x_expected << 1, 1, -1, 0, 0, 0;
  EXPECT_TRUE(CompareMatrices(result.GetSolution(x), x_expected, tol));
  EXPECT_NEAR(result.get_optimal_cost(), 1, tol);
}

void SolveSDPwithQuadraticCosts(const SolverInterface& solver, double tol) {
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<6>();
  Matrix3<symbolic::Variable> psd_mat1;
  Matrix3<symbolic::Variable> psd_mat2;
  // clang-format off
  psd_mat1 << x(0), x(1), x(3),
              x(1), x(0), x(4),
              x(3), x(4), x(5);
  psd_mat2 << x(0), x(2), x(3),
              x(2), x(0), x(4),
              x(3), x(4), x(5);
  // clang-format on
  auto psd_constraint1 = prog.AddPositiveSemidefiniteConstraint(psd_mat1);
  auto psd_constraint2 = prog.AddPositiveSemidefiniteConstraint(psd_mat2);
  prog.AddBoundingBoxConstraint(1, 1, x(1));
  prog.AddQuadraticCost(x(0) * x(0));
  prog.AddLinearCost(2 * x(0) + x(2) + x(5));

  MathematicalProgramResult result;
  solver.Solve(prog, {}, {}, &result);
  EXPECT_TRUE(result.is_success());
  Vector6<double> x_expected;
  x_expected << 1, 1, -1, 0, 0, 0;
  EXPECT_TRUE(CompareMatrices(result.GetSolution(x), x_expected, tol));
  EXPECT_NEAR(result.get_optimal_cost(), 2, tol);

  // Check the complementarity condition for the PSD constraint.
  const auto psd_dual1 = math::ToSymmetricMatrixFromLowerTriangularColumns(
      result.GetDualSolution(psd_constraint1));
  const auto psd_dual2 = math::ToSymmetricMatrixFromLowerTriangularColumns(
      result.GetDualSolution(psd_constraint2));
  const auto psd_mat1_sol = result.GetSolution(psd_mat1);
  const auto psd_mat2_sol = result.GetSolution(psd_mat2);
  EXPECT_NEAR((psd_dual1 * psd_mat1_sol).trace(), 0, tol);
  EXPECT_NEAR((psd_dual2 * psd_mat2_sol).trace(), 0, tol);
}

void TestSDPDualSolution1(const SolverInterface& solver, double tol,
                          double complementarity_tol) {
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<6>();
  Matrix3<symbolic::Variable> psd_mat;
  // clang-format off
  psd_mat << x(0), x(1), x(3),
             x(1), x(2), x(4),
             x(3), x(4), x(5);
  // clang-format on
  auto psd_con = prog.AddPositiveSemidefiniteConstraint(psd_mat);
  auto bb_con = prog.AddBoundingBoxConstraint(
      Eigen::Vector2d(kInf, kInf), Eigen::Vector2d(4, 1),
      Vector2<symbolic::Variable>(x(0), x(2)));
  prog.AddLinearCost(x(1) + x(5));
  MathematicalProgramResult result;
  solver.Solve(prog, {}, {}, &result);
  EXPECT_TRUE(result.is_success());

  const Vector6<double> x_sol = result.GetSolution(x);
  const auto psd_mat_sol = result.GetSolution(psd_mat);
  EXPECT_TRUE(CompareMatrices(
      x_sol, (Vector6d() << 4, -2, 1, 0, 0, 0).finished(), tol));
  // The optimal cost is -sqrt(x0 * x2), hence the sensitivity to the
  // bounding box constraint on x0 is -.25, and the sensitivity to the bounding
  // box constraint on x2 is -1.
  const Eigen::Vector2d bb_con_dual_expected(-0.25, -1);
  EXPECT_TRUE(CompareMatrices(result.GetDualSolution(bb_con),
                              bb_con_dual_expected, tol));
  const auto psd_dual = math::ToSymmetricMatrixFromLowerTriangularColumns(
      result.GetDualSolution(psd_con));
  // Complementarity condition ensures the inner product of psd_mat and its dual
  // is 0.
  EXPECT_NEAR((psd_mat_sol * psd_dual).trace(), 0, complementarity_tol);
  // The problem in the primal form is
  // min [0  0.5 0] ● X
  //     [0.5  0 0]
  //     [0    0 1]
  // s.t [1 0 0] ● X <= 4
  //     [0 0 0]
  //     [0 0 0]
  //
  //     [0 0 0] ● X <= 1
  //     [0 1 0]
  //     [0 0 0]
  // The problem in the dual form (LMI) is
  // max 4*y1 + y2
  // s.t [-y1  0.5 0]  is psd   (1)
  //     [0.5  -y2 0]
  //     [ 0   0   1]
  // The optimal solution is to the dual is y1 = -0.25, y2 = -1. Plug in this
  // dual solution to the left hand side of (1) is what Mosek/SCS returns as the
  // dual solution.
  // clang-format off
  Eigen::Matrix3d psd_dual_expected;
  psd_dual_expected << 0.25, 0.5, 0,
                       0.5, 1, 0,
                       0,  0, 1;
  // clang-format on
  EXPECT_TRUE(CompareMatrices(psd_dual, psd_dual_expected, tol));
}

void TestTrivial1x1SDP(const SolverInterface& solver, double primal_tol,
                       bool check_dual, double dual_tol) {
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<1>("x");
  auto scalar_psd_con =
      prog.AddPositiveSemidefiniteConstraint(Vector1<symbolic::Variable>(x(0)));
  prog.AddLinearCost(x(0));
  MathematicalProgramResult result;
  solver.Solve(prog, std::nullopt, std::nullopt, &result);
  EXPECT_TRUE(result.is_success());
  EXPECT_NEAR(result.get_optimal_cost(), 0, primal_tol);
  EXPECT_NEAR(result.GetSolution(x(0)), 0, primal_tol);
  if (check_dual) {
    const auto scalar_psd_dual =
        math::ToSymmetricMatrixFromLowerTriangularColumns(
            result.GetDualSolution(scalar_psd_con));
    EXPECT_TRUE(CompareMatrices(scalar_psd_dual, Vector1d(1), dual_tol));
  }
}

void TestTrivial2x2SDP(const SolverInterface& solver, double primal_tol,
                       bool check_dual, double dual_tol) {
  MathematicalProgram prog;
  auto S = prog.NewSymmetricContinuousVariables<2>();
  const auto psd_con = prog.AddPositiveSemidefiniteConstraint(S);
  prog.AddBoundingBoxConstraint(1, 1, S(1, 0));
  prog.AddLinearCost(S(0, 0) + S(1, 1));
  MathematicalProgramResult result;
  solver.Solve(prog, std::nullopt, std::nullopt, &result);
  EXPECT_TRUE(result.is_success());
  const Eigen::Matrix2d S_expected = Eigen::Matrix2d::Ones();
  EXPECT_TRUE(CompareMatrices(result.GetSolution(S), S_expected, primal_tol));
  EXPECT_NEAR(result.get_optimal_cost(), 2, primal_tol);
  if (check_dual) {
    Eigen::Matrix2d psd_dual_expected;
    // clang-format off
  psd_dual_expected << 1, -1,
                       -1, 1;
    // clang-format on
    const auto psd_dual = math::ToSymmetricMatrixFromLowerTriangularColumns(
        result.GetDualSolution(psd_con));
    EXPECT_TRUE(CompareMatrices(psd_dual, psd_dual_expected, dual_tol));
  }
}

void Test1x1with3x3SDP(const SolverInterface& solver, double primal_tol,
                       bool check_dual, double dual_tol) {
  MathematicalProgram prog;
  auto x = prog.NewSymmetricContinuousVariables<6>();
  const auto scalar_psd_con =
      prog.AddPositiveSemidefiniteConstraint(Vector1<symbolic::Variable>(x(1)));
  Matrix3<symbolic::Variable> psd_mat;
  // clang-format off
  psd_mat << x(0), x(1), x(3),
             x(1), x(2), x(4),
             x(3), x(4), x(5);
  // clang-format on
  const auto psd_con = prog.AddPositiveSemidefiniteConstraint(psd_mat);
  prog.AddBoundingBoxConstraint(9, 9, x(0));
  prog.AddBoundingBoxConstraint(1, 1, x(2));
  prog.AddBoundingBoxConstraint(4, 4, x(5));
  prog.AddLinearCost(x(1) + x(3) + x(4));
  MathematicalProgramResult result;
  solver.Solve(prog, std::nullopt, std::nullopt, &result);
  EXPECT_TRUE(result.is_success());
  const Eigen::Matrix3d psd_mat_sol = result.GetSolution(psd_mat);
  Eigen::Matrix3d psd_mat_expected;
  // The optimal solution occurs when x(1) = 0, and the problem is equivalent to
  // min x(3) + x(4).
  // s.t x(3)*x(3)+9*x(4)*x(4)<=36.
  // We can solve this problem analytically.
  // clang-format off
  psd_mat_expected << 9, 0, -18 / std::sqrt(10),
                      0, 1, -2 / std::sqrt(10),
                      -18 / std::sqrt(10), -2 / std::sqrt(10), 4;
  // clang-format on
  EXPECT_TRUE(CompareMatrices(psd_mat_sol, psd_mat_expected, primal_tol));
  EXPECT_NEAR(result.get_optimal_cost(), -20 / std::sqrt(10), primal_tol);
  if (check_dual) {
    const auto x_sol = result.GetSolution(x);
    // The dual problem is
    // max 9*y(0) + y(1) + 4*y(2)
    // s.t [-y(0)    0.5-0.5*y(3)   0.5] is psd
    //     [0.5-0.5*y(3)    -y(1)   0.5]
    //     [0.5              0.5  -y(2)]
    //     y(3) >= 0
    const Eigen::Matrix3d psd_dual =
        math::ToSymmetricMatrixFromLowerTriangularColumns(
            result.GetDualSolution(psd_con));
    const double scalar_psd_dual = result.GetDualSolution(scalar_psd_con)(0);
    EXPECT_NEAR(psd_dual(0, 1), 0.5 - 0.5 * scalar_psd_dual, dual_tol);
    // Check the duality gap, should be zero.
    EXPECT_NEAR(-9 * psd_dual(0, 0) - psd_dual(1, 1) - 4 * psd_dual(2, 2),
                result.get_optimal_cost(), dual_tol);
    // Check complementary slackness.
    EXPECT_NEAR((psd_mat_expected * psd_dual).trace(), 0, dual_tol);
    EXPECT_NEAR(x_sol(1) * scalar_psd_dual, 0, dual_tol);
    // Check the dual matrix being psd.
    EXPECT_GE(scalar_psd_dual, -dual_tol);
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(psd_dual);
    EXPECT_TRUE((es.eigenvalues().array() >= -dual_tol).all());
  }
}

void Test2x2with3x3SDP(const SolverInterface& solver, double primal_tol,
                       bool check_dual, double dual_tol) {
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<5>("x");
  prog.AddLinearCost(x(1) + x(2) + x(4));
  Matrix2<symbolic::Variable> psd_mat_2x2;
  Matrix3<symbolic::Variable> psd_mat_3x3;
  // clang-format off
  psd_mat_2x2 << x(0), x(1),
                 x(1), x(2);
  psd_mat_3x3 << x(0), x(1), x(2),
                 x(1), x(3), x(4),
                 x(2), x(4), x(2);
  // clang-format on
  auto psd_2x2_con = prog.AddPositiveSemidefiniteConstraint(psd_mat_2x2);
  auto psd_3x3_con = prog.AddPositiveSemidefiniteConstraint(psd_mat_3x3);
  prog.AddBoundingBoxConstraint(1, 1, x(0));
  prog.AddBoundingBoxConstraint(1, 1, x(3));
  MathematicalProgramResult result;
  solver.Solve(prog, std::nullopt, std::nullopt, &result);
  EXPECT_TRUE(result.is_success());
  const Eigen::Matrix2d psd_2x2_sol = result.GetSolution(psd_mat_2x2);
  const Eigen::Matrix3d psd_3x3_sol = result.GetSolution(psd_mat_3x3);
  Eigen::Matrix2d psd_2x2_expected;
  Eigen::Matrix3d psd_3x3_expected;
  // clang-format off
  psd_2x2_expected <<  1, -1,
                      -1,  1;
  psd_3x3_expected <<  1, -1,  1,
                      -1,  1, -1,
                       1, -1,  1;
  // clang-format on
  EXPECT_TRUE(CompareMatrices(psd_2x2_sol, psd_2x2_expected, primal_tol));
  EXPECT_TRUE(CompareMatrices(psd_3x3_sol, psd_3x3_expected, primal_tol));
  EXPECT_NEAR(result.get_optimal_cost(), -1, primal_tol);
  if (check_dual) {
    // The dual problem is
    // max y(4) + y(5)
    // [y(0)-y(4)  0.5 + 0.5*y(1)  0.5 + 0.5*y(2)-0.5*y(3)]
    // [    *         -y(5)           0.5                 ] is psd
    // [    *            *             y(3)               ]
    //
    // [-y(0) -0.5*y(1)] is psd
    // [ *      -y(2)  ]
    // The solution is y = [-0.5, -1, -0.5, 0.5, -0.5, -0.5]
    const auto psd_2x2_dual = math::ToSymmetricMatrixFromLowerTriangularColumns(
        result.GetDualSolution(psd_2x2_con));
    const auto psd_3x3_dual = math::ToSymmetricMatrixFromLowerTriangularColumns(
        result.GetDualSolution(psd_3x3_con));
    Eigen::Matrix2d psd_2x2_dual_expected;
    Eigen::Matrix3d psd_3x3_dual_expected;
    // clang-format off
    psd_2x2_dual_expected << 0.5, 0.5,
                             0.5, 0.5;
    psd_3x3_dual_expected << 0,   0,   0,
                             0, 0.5, 0.5,
                             0, 0.5, 0.5;
    // clang-format on
    EXPECT_TRUE(CompareMatrices(psd_2x2_dual, psd_2x2_dual_expected, dual_tol));
    EXPECT_TRUE(CompareMatrices(psd_3x3_dual, psd_3x3_dual_expected, dual_tol));
    // Check complementarity slackness.
    EXPECT_NEAR((psd_2x2_dual * psd_2x2_sol).trace(), 0, dual_tol);
    EXPECT_NEAR((psd_3x3_dual * psd_3x3_sol).trace(), 0, dual_tol);
  }
}

void TestTrivial1x1LMI(const SolverInterface& solver, double primal_tol,
                       bool check_dual, double dual_tol) {
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<2>();
  prog.AddLinearCost(x[0] + x[1]);
  auto lmi_con1 = prog.AddLinearMatrixInequalityConstraint(
      {Vector1d(1), Vector1d(2), Vector1d(-3)}, x);
  auto lmi_con2 = prog.AddLinearMatrixInequalityConstraint(
      {Vector1d(-3), Vector1d(-1), Vector1d(-3)}, x);
  auto lmi_con3 = prog.AddLinearMatrixInequalityConstraint(
      {Vector1d(4), Vector1d(1), Vector1d(2)}, x);
  MathematicalProgramResult result;
  solver.Solve(prog, std::nullopt, std::nullopt, &result);
  ASSERT_TRUE(result.is_success());
  const Eigen::Vector2d x_sol = result.GetSolution(x);
  EXPECT_TRUE(CompareMatrices(x_sol, Eigen::Vector2d(-2, -1), primal_tol));
  if (check_dual) {
    auto lmi_dual1 = result.GetDualSolution(lmi_con1);
    auto lmi_dual2 = result.GetDualSolution(lmi_con2);
    auto lmi_dual3 = result.GetDualSolution(lmi_con3);
    EXPECT_TRUE(CompareMatrices(lmi_dual1, Vector1d(1.0 / 7), dual_tol));
    EXPECT_TRUE(CompareMatrices(lmi_dual2, Vector1d(0), dual_tol));
    EXPECT_TRUE(CompareMatrices(lmi_dual3, Vector1d(5.0 / 7), dual_tol));
  }
}

void Test2x2LMI(const SolverInterface& solver, double primal_tol,
                bool check_dual, double dual_tol) {
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables(1);
  auto lmi_con = prog.AddLinearMatrixInequalityConstraint(
      {(Eigen::Matrix2d() << 1, 1, 1, -2).finished(),
       (Eigen::Matrix2d() << 2, 2, 2, 8).finished()},
      x);
  prog.AddLinearCost(x(0));
  MathematicalProgramResult result;
  solver.Solve(prog, std::nullopt, std::nullopt, &result);
  ASSERT_TRUE(result.is_success());
  const auto x_sol = result.GetSolution(x);
  EXPECT_TRUE(CompareMatrices(x_sol, Vector1d(0.5), primal_tol));
  EXPECT_NEAR(result.get_optimal_cost(), 0.5, primal_tol);
  if (check_dual) {
    const Eigen::Matrix2d lmi_dual =
        math::ToSymmetricMatrixFromLowerTriangularColumns(
            result.GetDualSolution(lmi_con));
    const Eigen::Matrix2d lmi_sol =
        lmi_con.evaluator()->F()[0] + lmi_con.evaluator()->F()[1] * x_sol(0);
    EXPECT_NEAR((lmi_dual * lmi_sol).trace(), 0, dual_tol);
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> es(lmi_dual);
    EXPECT_TRUE((es.eigenvalues().array() >= -dual_tol).all());
  }
}

namespace {
MatrixX<symbolic::Variable> Hankel(
    const Eigen::Ref<const VectorX<symbolic::Variable>>& c,
    const Eigen::Ref<const VectorX<symbolic::Variable>>& r) {
  MatrixX<symbolic::Variable> ret(c.rows(), r.rows());
  for (int i = 0; i < ret.rows(); ++i) {
    for (int j = 0; j < ret.cols(); ++j) {
      if (i + j < ret.rows()) {
        ret(i, j) = c(i + j);
      } else {
        ret(i, j) = r(i + j - ret.rows() + 1);
      }
    }
  }
  return ret;
}
}  // namespace
void TestHankel(const SolverInterface& solver, double primal_tol,
                bool check_dual, double dual_tol) {
  int n = 3;
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables(2 * n - 1, "x");
  auto X = Hankel(x.head(n), x.tail(n));
  auto psd_con = prog.AddPositiveSemidefiniteConstraint(X);
  prog.AddBoundingBoxConstraint(1, kInf, x(1));
  prog.AddLinearCost(X.cast<symbolic::Expression>().trace());

  MathematicalProgramResult result;
  solver.Solve(prog, std::nullopt, std::nullopt, &result);
  ASSERT_TRUE(result.is_success());

  const auto X_sol = result.GetSolution(X);
  // We can compute the optimal solution by hand.
  const double x2_expected = std::sqrt((std::sqrt(13) - 1) / 6);
  const double x0_expected = 1 / x2_expected;
  const double x3_expected = x2_expected * x2_expected;
  const double x4_expected = x3_expected * x3_expected / x2_expected;
  Eigen::Matrix<double, 5, 1> x_expected;
  x_expected << x0_expected, 1, x2_expected, x3_expected, x4_expected;
  const auto x_sol = result.GetSolution(x);
  EXPECT_TRUE(CompareMatrices(x_sol, x_expected, primal_tol));
  Eigen::Matrix3d X_expected;
  // clang-format off
  X_expected << x0_expected, 1, x2_expected,
                1, x2_expected, x3_expected,
                x2_expected, x3_expected, x4_expected;
  // clang-format on
  EXPECT_TRUE(CompareMatrices(X_sol, X_expected, primal_tol));
  if (check_dual) {
    // The dual PSD matrix has the form
    // [1 -y1/2 -y0/2]
    // [* 1+y0    0  ]
    // [*   *     1  ]
    // with y1 >= 0
    const Eigen::Matrix3d psd_dual =
        math::ToSymmetricMatrixFromLowerTriangularColumns(
            result.GetDualSolution(psd_con));
    EXPECT_NEAR((psd_dual * X_sol).trace(), 0, dual_tol);
    EXPECT_NEAR(psd_dual(0, 0), 1, dual_tol);
    EXPECT_NEAR(psd_dual(1, 2), 0, dual_tol);
    EXPECT_NEAR(psd_dual(2, 2), 1, dual_tol);
    EXPECT_LE(psd_dual(0, 1), dual_tol);
    EXPECT_NEAR(psd_dual(0, 2) * 2 + psd_dual(1, 1), 1, primal_tol);

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(psd_dual);
    EXPECT_TRUE((es.eigenvalues().array() >= -dual_tol).all());
  }
}

}  // namespace test
}  // namespace solvers
}  // namespace drake
