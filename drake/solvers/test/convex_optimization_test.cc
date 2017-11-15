#include <iostream>
#include <list>
#include <memory>

#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/solvers/gurobi_solver.h"
#include "drake/solvers/mosek_solver.h"
#include "drake/solvers/test/add_solver_util.h"
#include "drake/solvers/test/mathematical_program_test_util.h"

using std::make_shared;

using Eigen::Vector2d;
using Eigen::Matrix2d;
using Eigen::Matrix3d;
using Eigen::Vector3d;
using Eigen::Vector4d;
using Eigen::Matrix4d;
using Eigen::RowVector2d;

namespace drake {
namespace solvers {
namespace test {
namespace {
void GetSecondOrderConicProgramSolvers(
    std::list<std::unique_ptr<MathematicalProgramSolverInterface>>* solvers) {
  AddSolverIfAvailable<GurobiSolver>(solvers);
  AddSolverIfAvailable<MosekSolver>(solvers);
}

void GetSemidefiniteProgramSolvers(
    std::list<std::unique_ptr<MathematicalProgramSolverInterface>>* solvers) {
  AddSolverIfAvailable<MosekSolver>(solvers);
}

////////////////////////////////////////
//// Second order conic program ////////
////////////////////////////////////////

// This example is taken from the paper
// Applications of second-order cone programming
// By M.S.Lobo, L.Vandenberghe, S.Boyd and H.Lebret,
// Section 3.6
// http://www.seas.ucla.edu/~vandenbe/publications/socp.pdf
// The problem tries to find the equilibrium state of a mechanical
// system, which consists of N nodes at position (x1,y1), (x2,y2), ..., (xN,yN)
// in R2.
// The nodes are connected by springs with given coefficient.
// The spring generate force when it is stretched,
// but not when it is compressed.
// Namely, the spring force is
// (spring_length - spring_rest_length) * spring_stiffness,
// if spring_length >= spring_rest_length;
// otherwise the spring force is zero.
// weight_i is the mass * gravity_acceleration
// of the i'th link.
// The equilibrium point is obtained when the total energy is minimized
// namely min sum_i weight_i * yi + stiffness/2 * t_i^2
//        s.t  sqrt((x(i) - x(i+1))^2 + (y(i) - y(i+1))^2) - spring_rest_length
// <= t_i
//             0 <= t_i
//             (x1,y1) = end_pos1
//             (xN,yN) = end_pos2
// By introducing a slack variable z >= t_1^2 + ... + t_(N-1)^2, the problem
// becomes
// an SOCP, with both Lorentz cone and rotated Lorentz cone constraints
void FindSpringEquilibrium(const Eigen::VectorXd& weight,
                           double spring_rest_length, double spring_stiffness,
                           const Eigen::Vector2d& end_pos1,
                           const Eigen::Vector2d& end_pos2,
                           const MathematicalProgramSolverInterface& solver) {
  int num_nodes = weight.rows();
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables(num_nodes, "x");
  auto y = prog.NewContinuousVariables(num_nodes, "y");
  auto t = prog.NewContinuousVariables(num_nodes - 1, "t");
  prog.AddBoundingBoxConstraint(end_pos1, end_pos1, {x.head<1>(), y.head<1>()});
  prog.AddBoundingBoxConstraint(
      end_pos2, end_pos2,
      {x.segment<1>(num_nodes - 1), y.segment<1>(num_nodes - 1)});
  prog.AddBoundingBoxConstraint(
      Eigen::VectorXd::Zero(num_nodes - 1),
      Eigen::VectorXd::Constant(num_nodes - 1,
                                std::numeric_limits<double>::infinity()),
      t);

  // sqrt((x(i)-x(i+1))^2 + (y(i) - y(i+1))^2) <= ti + spring_rest_length
  for (int i = 0; i < num_nodes - 1; ++i) {
    // A_lorentz1 * [x(i); x(i+1); y(i); y(i+1); t(i)] + b_lorentz1
    //     = [ti + spring_rest_length; x(i) - x(i+1); y(i) - y(i+1)]
    Eigen::Matrix<double, 3, 5> A_lorentz1;
    A_lorentz1.setZero();
    A_lorentz1(0, 4) = 1;
    A_lorentz1(1, 0) = 1;
    A_lorentz1(1, 1) = -1;
    A_lorentz1(2, 2) = 1;
    A_lorentz1(2, 3) = -1;
    Eigen::Vector3d b_lorentz1(spring_rest_length, 0, 0);
    prog.AddLorentzConeConstraint(
        A_lorentz1, b_lorentz1,
        {x.segment<2>(i), y.segment<2>(i), t.segment<1>(i)});
  }

  // Add constraint z >= t_1^2 + .. + t_(N-1)^2
  auto z = prog.NewContinuousVariables<1>("z");
  Eigen::MatrixXd A_lorentz2(1 + num_nodes, num_nodes);
  A_lorentz2 << Eigen::RowVectorXd::Zero(num_nodes),
      Eigen::MatrixXd::Identity(num_nodes, num_nodes);
  Eigen::VectorXd b_lorentz2(1 + num_nodes);
  b_lorentz2 << 1, Eigen::VectorXd::Zero(num_nodes);
  prog.AddRotatedLorentzConeConstraint(A_lorentz2, b_lorentz2, {z, t});

  prog.AddLinearCost(drake::Vector1d(spring_stiffness / 2), z);
  prog.AddLinearCost(weight, y);

  RunSolver(&prog, solver);

  const optional<SolverId> solver_id = prog.GetSolverId();
  ASSERT_TRUE(solver_id);
  double precision = 1e-3;
  // The precision of Gurobi solver is not as good as Mosek, in this problem.
  if (*solver_id == GurobiSolver::id()) {
    precision = 2e-2;
  }
  for (int i = 0; i < num_nodes - 1; ++i) {
    Eigen::Vector2d spring(prog.GetSolution(x(i + 1)) - prog.GetSolution(x(i)),
                           prog.GetSolution(y(i + 1)) - prog.GetSolution(y(i)));
    if (spring.norm() < spring_rest_length) {
      EXPECT_LE(prog.GetSolution(t(i)), 1E-3);
      EXPECT_GE(prog.GetSolution(t(i)), 0 - 1E-10);
    } else {
      EXPECT_TRUE(std::abs(spring.norm() - spring_rest_length -
                           prog.GetSolution(t(i))) < 1E-3);
    }
  }
  const auto& t_value = prog.GetSolution(t);
  EXPECT_TRUE(std::abs(prog.GetSolution(z(0)) - t_value.squaredNorm()) < 1E-3);
  // Now test equilibrium.
  for (int i = 1; i < num_nodes - 1; i++) {
    Eigen::Vector2d left_spring(
        prog.GetSolution(x(i - 1)) - prog.GetSolution(x(i)),
        prog.GetSolution(y(i - 1)) - prog.GetSolution(y(i)));
    Eigen::Vector2d left_spring_force;
    double left_spring_length = left_spring.norm();
    if (left_spring_length < spring_rest_length) {
      left_spring_force.setZero();
    } else {
      left_spring_force = (left_spring_length - spring_rest_length) *
                          spring_stiffness * left_spring / left_spring_length;
    }
    Eigen::Vector2d right_spring(
        prog.GetSolution(x(i + 1)) - prog.GetSolution(x(i)),
        prog.GetSolution(y(i + 1)) - prog.GetSolution(y(i)));
    Eigen::Vector2d right_spring_force;
    double right_spring_length = right_spring.norm();
    if (right_spring_length < spring_rest_length) {
      right_spring_force.setZero();
    } else {
      right_spring_force = (right_spring_length - spring_rest_length) *
                           spring_stiffness * right_spring /
                           right_spring_length;
    }
    Eigen::Vector2d weight_i(0, -weight(i));
    EXPECT_TRUE(CompareMatrices(
        weight_i + left_spring_force + right_spring_force,
        Eigen::Vector2d::Zero(), precision, MatrixCompareType::absolute));
  }
}

void TestFindSpringEquilibrium(
    const MathematicalProgramSolverInterface& solver) {
  Eigen::VectorXd weight(5);
  weight << 1, 2, 3, 2.5, 4;
  double spring_rest_length = 0.2;
  double spring_stiffness_coefficient = 10;
  Eigen::Vector2d end_pos1(0, 1);
  Eigen::Vector2d end_pos2(1, 0.9);

  FindSpringEquilibrium(weight, spring_rest_length,
                        spring_stiffness_coefficient, end_pos1, end_pos2,
                        solver);
}
}  // namespace

GTEST_TEST(TestSOCP, TestFindSpringEquilibrium) {
  std::list<std::unique_ptr<MathematicalProgramSolverInterface>> solvers;
  GetSecondOrderConicProgramSolvers(&solvers);
  for (const auto& solver : solvers) {
    TestFindSpringEquilibrium(*solver);
  }
}

// Test a trivial semidefinite problem.
// min S(0, 0) + S(1, 1)
// s.t S(1, 0) = 1
//     S is p.s.d
// The analytical solution is
// S = [1 1]
//     [1 1]
GTEST_TEST(TestSemidefiniteProgram, TestTrivialSDP) {
  std::list<std::unique_ptr<MathematicalProgramSolverInterface>> solvers;
  GetSemidefiniteProgramSolvers(&solvers);
  for (const auto& solver : solvers) {
    MathematicalProgram prog;

    auto S = prog.NewSymmetricContinuousVariables<2>("S");

    prog.AddPositiveSemidefiniteConstraint(S);

    prog.AddBoundingBoxConstraint(1, 1, S(1, 0));

    prog.AddLinearCost(Eigen::Vector2d(1, 1), S.diagonal());

    RunSolver(&prog, *solver);

    auto S_value = prog.GetSolution(S);

    // Choose 1E-8 since that is Mosek's default feasibility tolerance.
    EXPECT_TRUE(CompareMatrices(S_value, Eigen::Matrix2d::Ones(), 1E-8));
  }
}

// Solve a semidefinite programming problem.
// Find the common Lyapunov function for linear systems
// xdot = Ai*x
// The condition is
// min 0
// s.t P is positive definite
//     - (Ai'*P + P*Ai) is positive definite
GTEST_TEST(TestSemidefiniteProgram, TestCommonLyapunov) {
  std::list<std::unique_ptr<MathematicalProgramSolverInterface>> solvers;
  GetSemidefiniteProgramSolvers(&solvers);
  for (const auto& solver : solvers) {
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

    RunSolver(&prog, *solver);

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
}

/*
 * Given some ellipsoids ℰᵢ : xᵀ*Qi*x + 2 * biᵀ * x <= 1, i = 1, 2, ..., n,
 * find an ellipsoid
 * xᵀ*P*x + 2*cᵀ*x <= 1 as an outer approximation for the union of ellipsoids
 * ℰᵢ.
 * Using s-lemma, the ellipsoid xᵀ*P*x + 2*cᵀ*x <= 1 contains the ellipsoid
 * ℰᵢ : xᵀ*Qi*x + 2*biᵀ*x <= 1, if and only if there exists a scalar si >= 0
 * and (1 - xᵀ*P*x - cᵀ * x) - si*(1 - xᵀ*Qi*x - biᵀ * x) >= 0 ∀x.
 * Namely the matrix
 * [ si*Qi - P    si*bi - c]
 * [si*biᵀ - cᵀ      1 - si]
 * is positive semidefinite.
 * In order to find a tight outer approximation, we choose to minimize the
 * trace of P.
 * The optimiation problem becomes
 * min_{P, c, si} -trace(P)
 * s.t [ si*Qi - P    si*bi - c] is p.s.d
 *     [si*biᵀ - cᵀ      1 - si]
 *     P is p.s.d
 */
GTEST_TEST(TestSemidefiniteProgram, TestOuterEllipsoid) {
  std::list<std::unique_ptr<MathematicalProgramSolverInterface>> solvers;
  GetSemidefiniteProgramSolvers(&solvers);
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
  for (const auto& solver : solvers) {
    MathematicalProgram prog;
    auto P = prog.NewSymmetricContinuousVariables<3>("P");
    prog.AddPositiveSemidefiniteConstraint(P);
    auto s = prog.NewContinuousVariables<3>("s");
    prog.AddBoundingBoxConstraint(0, std::numeric_limits<double>::infinity(),
                                  s);
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

    RunSolver(&prog, *solver);

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
}

// Solve an eigen value problem through a semidefinite programming.
// Minimize the maximum eigen value of a matrix that depends affinely on a
// variable x
// min  z
// s.t z * Identity - x1 * F1 - ... - xn * Fn is p.s.d
//     A * x <= b
//     C * x = d
GTEST_TEST(TestSemidefiniteProgram, TestEigenvalueProblem) {
  std::list<std::unique_ptr<MathematicalProgramSolverInterface>> solvers;
  GetSemidefiniteProgramSolvers(&solvers);
  for (const auto& solver : solvers) {
    MathematicalProgram prog;
    auto x = prog.NewContinuousVariables<2>("x");
    Matrix3d F1;
    // clang-format off
    F1 << 1, 0.2, 0.3,
          0.2, 2, -0.1,
          0.3, -0.1, 4;
    // clang-format on
    Matrix3d F2;
    // clang-format off
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

    RunSolver(&prog, *solver);

    double z_value = prog.GetSolution(z(0));
    auto x_value = prog.GetSolution(x);
    auto xF_sum = x_value(0) * F1 + x_value(1) * F2;

    Eigen::SelfAdjointEigenSolver<Matrix3d> eigen_solver_xF(xF_sum);
    // The comparison tolerance is set to 1E-7, slightly larger than Mosek's
    // default feasibility tolerance 1E-8.
    EXPECT_NEAR(z_value, eigen_solver_xF.eigenvalues().maxCoeff(), 1E-7);
  }
}
}  // namespace test
}  // namespace solvers
}  // namespace drake
