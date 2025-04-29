#include "drake/solvers/projected_gradient_descent_solver.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/solve.h"

namespace drake {
namespace solvers {
namespace test {

using Eigen::Vector2d;
using Eigen::VectorXd;

GTEST_TEST(ProjectedGradientDescentSolverTest, QP) {
  // First program is unconstrained
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<2>();
  prog.AddCost(pow(x(0) - 1, 2) + pow(x(1) - 1, 2));

  ProjectedGradientDescentSolver solver;
  MathematicalProgramResult result = solver.Solve(prog, {}, {});

  VectorXd expected_answer = Vector2d(1.0, 1.0);
  auto x_value = result.GetSolution(x);
  EXPECT_TRUE(CompareMatrices(expected_answer, x_value, 1e-4,
                              MatrixCompareType::absolute));
  EXPECT_NEAR(0.0, result.get_optimal_cost(), 1e-4);

  // Add a linear equality constraint.
  prog.AddConstraint(x(0) == 2.0);
  result = solver.Solve(prog, {}, {});
  expected_answer = Vector2d(2.0, 1.0);
  x_value = result.GetSolution(x);
  EXPECT_TRUE(CompareMatrices(expected_answer, x_value, 1e-4,
                              MatrixCompareType::absolute));
  EXPECT_NEAR(1.0, result.get_optimal_cost(), 1e-4);

  // Add a linear inequality constraint.
  prog.AddConstraint(x(1) <= 0.0);
  result = solver.Solve(prog, {}, {});
  expected_answer = Vector2d(2.0, 0.0);
  x_value = result.GetSolution(x);
  EXPECT_TRUE(CompareMatrices(expected_answer, x_value, 1e-4,
                              MatrixCompareType::absolute));
  EXPECT_NEAR(2.0, result.get_optimal_cost(), 1e-4);
}

GTEST_TEST(ProjectedGradientDescentSolverTest, DifferentInitialGuesses) {
  // Program with one variable, x. Require that x^2 >= 4 (so x >= 2 or x <= -2).
  // Try to minimize (x-1)^2, so we have a local minimum at x=-2 with cost 9 and
  // a global minimum at x=2 with cost 1.
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<1>();
  prog.AddCost(pow(x(0) - 1, 2));
  prog.AddConstraint(pow(x(0), 2) >= 4);

  ProjectedGradientDescentSolver solver;
  MathematicalProgramResult result;
  VectorXd x_value;

  result = solver.Solve(prog, Vector1d(-1.0), {});
  x_value = result.GetSolution(x);
  EXPECT_NEAR(x_value[0], -2.0, 1e-4);
  EXPECT_NEAR(result.get_optimal_cost(), 9.0, 1e-4);

  result = solver.Solve(prog, Vector1d(1.0), {});
  x_value = result.GetSolution(x);
  EXPECT_NEAR(x_value[0], 2.0, 1e-4);
  EXPECT_NEAR(result.get_optimal_cost(), 1.0, 1e-4);
}

}  // namespace test
}  // namespace solvers
}  // namespace drake
