// This is to test a QP problem solved by both SNOPT and Gurobi
#include "drake/solvers/gurobi_solver.h"
#include "drake/solvers/snopt_solver.h"

#include "gtest/gtest.h"

#include "drake/common/eigen_matrix_compare.h"

namespace drake {
namespace solvers {
namespace {
/*
 * Test a constrained QP, with both equality and inequality constraitns
 * on a subset of decision variables.
 * The parameters of this QP are randomly generated.
 */
GTEST_TEST(testGurobi, checkGurobiQPAgainstSnopt) {
  MathematicalProgram prog;
  auto x = prog.AddContinuousVariables(2, "x");
  auto z = prog.AddContinuousVariables(2, "z");
  auto y = prog.AddContinuousVariables(2, "y");

  // Randomly generated vector.
  Eigen::Matrix<double, 2, 4> CE_xy;
  CE_xy << 0.3147, 0.1324, 0.4575, 0.4572, 0.4058, -0.4025, 0.4649, -0.0146;
  Eigen::Matrix<double, 2, 1> ce0_xy;
  ce0_xy << -0.0782, 0.4157;

  Eigen::Matrix<double, 1, 4> CI_xy;
  CI_xy << 0.1797, 0.0853, 0.4593, 0.3143;
  Eigen::Matrix<double, 1, 1> ci_xy_upper, ci_xy_lower;
  ci_xy_upper << -10;
  ci_xy_lower << -std::numeric_limits<double>::infinity();

  Eigen::Matrix<double, 1, 2> CI_z;
  CI_z << -0.3639, 0.3693;
  Eigen::Matrix<double, 1, 1> ci_z_upper, ci_z_lower;
  ci_z_upper << -10;
  ci_z_lower << -std::numeric_limits<double>::infinity();

  // Randomly generated matrix.
  Eigen::Matrix<double, 6, 6> G;
  G << 0.6472, -0.1517, -0.1019, -0.4394, 0.1772, 0.1697, -0.1517, 0.3755,
      0.0967, 0.3431, 0.2979, 0.1124, -0.1019, 0.0967, 0.1902, 0.0987, 0.2064,
      -0.0820, -0.4394, 0.3431, 0.0987, 0.6460, 0.1524, 0.0546, 0.1772, 0.2979,
      0.2064, 0.1524, 1.0408, 0.2175, 0.1697, 0.1124, -0.0820, 0.0546, 0.2175,
      0.5526;

  Eigen::Matrix<double, 6, 1> g0;
  g0 << 0.3001, -0.0686, 0.4106, -0.3182, -0.2362, -0.3545;

  prog.AddQuadraticCost(G, g0, {x, z, y});
  prog.AddLinearEqualityConstraint(CE_xy, ce0_xy, {x, y});
  prog.AddLinearConstraint(CI_xy, ci_xy_lower, ci_xy_upper, {x, y});
  prog.AddLinearConstraint(CI_z, ci_z_lower, ci_z_upper, {z});

  SolutionResult result;
  SnoptSolver snopt;
  result = snopt.Solve(prog);
  EXPECT_TRUE(result == SolutionResult::kSolutionFound);

  Eigen::VectorXd snopt_solution = prog.GetSolutionVectorValues();

  GurobiSolver gurobi;
  result = gurobi.Solve(prog);
  EXPECT_TRUE(result == SolutionResult::kSolutionFound);

  Eigen::VectorXd gurobi_solution = prog.GetSolutionVectorValues();

  EXPECT_TRUE(CompareMatrices(gurobi_solution, snopt_solution, 1e-4,
                              MatrixCompareType::absolute));
}
}  // namespace
}  // namespace solvers
}  // namespace drake
