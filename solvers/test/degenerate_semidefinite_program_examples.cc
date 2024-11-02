#include "drake/solvers/test/degenerate_semidefinite_program_examples.h"

#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/math/matrix_util.h"

namespace drake {
namespace solvers {
namespace test {
void TestTrivial1x1SDP(const SolverInterface& solver, double tol,
                       bool check_dual) {
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<1>("x");
  auto scalar_psd_con =
      prog.AddPositiveSemidefiniteConstraint(Vector1<symbolic::Variable>(x(0)));
  prog.AddLinearCost(x(0));
  MathematicalProgramResult result;
  solver.Solve(prog, std::nullopt, std::nullopt, &result);
  EXPECT_TRUE(result.is_success());
  EXPECT_NEAR(result.get_optimal_cost(), 0, tol);
  EXPECT_NEAR(result.GetSolution(x(0)), 0, tol);
  if (check_dual) {
    const auto scalar_psd_dual =
        math::ToSymmetricMatrixFromLowerTriangularColumns(
            result.GetDualSolution(scalar_psd_con));
    EXPECT_TRUE(CompareMatrices(scalar_psd_dual, Vector1d(1), tol));
  }
}

void TestTrivial2x2SDP(const SolverInterface& solver, double tol,
                       bool check_dual) {
  MathematicalProgram prog;
  auto S = prog.NewSymmetricContinuousVariables<2>();
  const auto psd_con = prog.AddPositiveSemidefiniteConstraint(S);
  prog.AddBoundingBoxConstraint(1, 1, S(1, 0));
  prog.AddLinearCost(S(0, 0) + S(1, 1));
  MathematicalProgramResult result;
  solver.Solve(prog, std::nullopt, std::nullopt, &result);
  EXPECT_TRUE(result.is_success());
  const Eigen::Matrix2d S_expected = Eigen::Matrix2d::Ones();
  EXPECT_TRUE(CompareMatrices(result.GetSolution(S), S_expected, tol));
  EXPECT_NEAR(result.get_optimal_cost(), 2, tol);
  if (check_dual) {
    Eigen::Matrix2d psd_dual_expected;
    // clang-format off
  psd_dual_expected << 1, -1,
                       -1, 1;
    // clang-format on
    const auto psd_dual = math::ToSymmetricMatrixFromLowerTriangularColumns(
        result.GetDualSolution(psd_con));
    EXPECT_TRUE(CompareMatrices(psd_dual, psd_dual_expected, tol));
  }
}

void Test1x1with3x3SDP(const SolverInterface& solver, double tol,
                       bool check_dual) {
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
  EXPECT_TRUE(CompareMatrices(psd_mat_sol, psd_mat_expected, tol));
  EXPECT_NEAR(result.get_optimal_cost(), -20 / std::sqrt(10), tol);
  if (check_dual) {
    const auto x_sol = result.GetSolution(x);
    // The dual problem is
    // max 9*y(0) + y(1)+4*y(2)
    // s.t [-y(0)    0.5-0.5*y(3)   0.5] is psd
    //     [0.5-0.5*y(3)    -y(1)   0.5]
    //     [0.5              0.5  -y(2)]
    //     y(3) >= 0
    const Eigen::Matrix3d psd_dual =
        math::ToSymmetricMatrixFromLowerTriangularColumns(
            result.GetDualSolution(psd_con));
    const double scalar_psd_dual = result.GetDualSolution(scalar_psd_con)(0);
    EXPECT_NEAR(psd_dual(0, 1), 0.5 - 0.5 * scalar_psd_dual, tol);
    // Check the duality gap, should be zero.
    EXPECT_NEAR(-9 * psd_dual(0, 0) - psd_dual(1, 1) - 4 * psd_dual(2, 2),
                result.get_optimal_cost(), tol);
    // Check complementarity slackness.
    EXPECT_NEAR((psd_mat_expected * psd_dual).trace(), 0, tol);
    EXPECT_NEAR(x_sol(1) * scalar_psd_dual, 0, tol);
    // Check the dual matrix being psd.
    EXPECT_GE(scalar_psd_dual, -tol);
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(psd_dual);
    EXPECT_TRUE((es.eigenvalues().array() >= -tol).all());
  }
}

void Test2x2with3x3SDP(const SolverInterface& solver, double tol,
                       bool check_dual) {
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
  EXPECT_TRUE(CompareMatrices(psd_2x2_sol, psd_2x2_expected, tol));
  EXPECT_TRUE(CompareMatrices(psd_3x3_sol, psd_3x3_expected, tol));
  EXPECT_NEAR(result.get_optimal_cost(), -1, tol);
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
    EXPECT_TRUE(CompareMatrices(psd_2x2_dual, psd_2x2_dual_expected, tol));
    EXPECT_TRUE(CompareMatrices(psd_3x3_dual, psd_3x3_dual_expected, tol));
    // Check complementarity slackness.
    EXPECT_NEAR((psd_2x2_dual * psd_2x2_sol).trace(), 0, tol);
    EXPECT_NEAR((psd_3x3_dual * psd_3x3_sol).trace(), 0, tol);
  }
}

}  // namespace test
}  // namespace solvers
}  // namespace drake
