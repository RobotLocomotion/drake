#include "drake/solvers/projected_gradient_descent_solver.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/solvers/equality_constrained_qp_solver.h"
#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/solve.h"

namespace drake {
namespace solvers {
namespace test {

using Eigen::Vector2d;
using Eigen::VectorXd;

class SimpleUnconstrainedQP : public ::testing::Test {
 protected:
  SimpleUnconstrainedQP() {
    x_ = prog_.NewContinuousVariables<1>();
    prog_.AddCost(pow(x_(0), 2));
  }
  Vector1<symbolic::Variable> x_;
  MathematicalProgram prog_;
  ProjectedGradientDescentSolver solver_;
};

GTEST_TEST(ProjectedGradientDescentSolverTest, QP) {
  // First program is unconstrained
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<2>();
  prog.AddCost(pow(x(0) - 1, 2) + pow(x(1) - 1, 2));

  ProjectedGradientDescentSolver solver;
  MathematicalProgramResult result = solver.Solve(prog, {}, {});
  EXPECT_TRUE(result.is_success());
  VectorXd expected_answer = Vector2d(1.0, 1.0);
  auto x_value = result.GetSolution(x);
  EXPECT_TRUE(CompareMatrices(expected_answer, x_value, 1e-4,
                              MatrixCompareType::absolute));
  EXPECT_NEAR(0.0, result.get_optimal_cost(), 1e-4);

  // Add a linear equality constraint.
  prog.AddConstraint(x(0) == 2.0);
  result = solver.Solve(prog, {}, {});
  EXPECT_TRUE(result.is_success());
  expected_answer = Vector2d(2.0, 1.0);
  x_value = result.GetSolution(x);
  EXPECT_TRUE(CompareMatrices(expected_answer, x_value, 1e-4,
                              MatrixCompareType::absolute));
  EXPECT_NEAR(1.0, result.get_optimal_cost(), 1e-4);

  // Add a linear inequality constraint.
  prog.AddConstraint(x(1) <= 0.0);
  result = solver.Solve(prog, {}, {});
  EXPECT_TRUE(result.is_success());
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
  EXPECT_TRUE(result.is_success());
  x_value = result.GetSolution(x);
  EXPECT_NEAR(x_value[0], -2.0, 1e-4);
  EXPECT_NEAR(result.get_optimal_cost(), 9.0, 1e-4);

  result = solver.Solve(prog, Vector1d(1.0), {});
  EXPECT_TRUE(result.is_success());
  x_value = result.GetSolution(x);
  EXPECT_NEAR(x_value[0], 2.0, 1e-4);
  EXPECT_NEAR(result.get_optimal_cost(), 1.0, 1e-4);
}

GTEST_TEST(ProjectedGradientDescentSolverTest, CustomProjectionFunction) {
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<1>();
  prog.AddCost(pow(x(0), 2));

  // Note that we don't add any constraints to the program, to ensure the custom
  // projection function is being used. We restrict 1 <= x <= 3, and minimize
  // x^2.
  auto custom_projection_function = [](const Vector1d& y) {
    return Vector1d(std::max(1.0, std::min(y[0], 3.0)));
  };

  ProjectedGradientDescentSolver solver;
  solver.SetCustomProjectionFunction(custom_projection_function);

  // We deliberately give it an infeasible initial guess to make sure the custom
  // projection function is being called properly.
  MathematicalProgramResult result = solver.Solve(prog, Vector1d(4.0), {});
  EXPECT_TRUE(result.is_success());
  auto x_value = result.GetSolution(x);
  EXPECT_NEAR(x_value[0], 1.0, 1e-4);
  EXPECT_NEAR(result.get_optimal_cost(), 1.0, 1e-4);
}

GTEST_TEST(ProjectedGradientDescentSolverTest, CustomGradientFunction) {
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<1>();
  prog.AddCost(pow(x(0), 2));
  prog.AddConstraint(x(0) >= 1.0);
  prog.AddConstraint(x(0) <= 3.0);

  auto custom_gradient_function = [](const Vector1d& y) {
    return Vector1d(2.0 * y);
  };

  ProjectedGradientDescentSolver solver;
  solver.SetCustomGradientFunction(custom_gradient_function);

  // We deliberately give it an infeasible initial guess to make sure the custom
  // projection function is being called properly.
  MathematicalProgramResult result = solver.Solve(prog, Vector1d(4.0), {});
  EXPECT_TRUE(result.is_success());
  auto x_value = result.GetSolution(x);
  EXPECT_NEAR(x_value[0], 1.0, 1e-4);
  EXPECT_NEAR(result.get_optimal_cost(), 1.0, 1e-4);
}

GTEST_TEST(ProjectedGradientDescentSolverTest, ProjectionSolverInterface) {
  // Verify that ProjectedGradientDescentSolver uses the provided
  // ProjectionSolverInterface, by providing an invalid one, and catching the
  // error for an unsupported program type.
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<1>();
  prog.AddConstraint(pow(x(0), 2) >= 1);  // Generic nonlinear constraint.

  ProjectedGradientDescentSolver solver;
  EqualityConstrainedQPSolver projection_solver;
  solver.SetProjectionSolverInterface(&projection_solver);

  DRAKE_EXPECT_THROWS_MESSAGE(
      solver.Solve(prog, {}, {}),
      ".*EqualityConstrainedQPSolver is unable to solve.*");
}

TEST_F(SimpleUnconstrainedQP, FeasibilityTol) {
  prog_.AddConstraint(x_(0) <= 0.0);

  // Custom projection function that always returns 1e-3
  auto custom_projection_function = [](const Vector1d& y) {
    return Vector1d(1e-3);
  };

  solver_.SetCustomProjectionFunction(custom_projection_function);

  SolverOptions options;
  options.SetOption(ProjectedGradientDescentSolver::id(),
                    ProjectedGradientDescentSolver::FeasibilityTolOptionName(),
                    1e-2);

  // The default feasibility tolerance of 1e-4 will fail. The custom tolerance
  // of 1e-2 will succeed.
  MathematicalProgramResult result =
      solver_.Solve(prog_, Vector1d(1.0 + 1e-3), {});
  EXPECT_FALSE(result.is_success());
  EXPECT_EQ(result.get_solution_result(),
            SolutionResult::kInfeasibleConstraints);
  result = solver_.Solve(prog_, Vector1d(1.0 + 1e-3), options);
  EXPECT_TRUE(result.is_success());
}

TEST_F(SimpleUnconstrainedQP, ConvergenceTol) {
  SolverOptions options;
  options.SetOption(ProjectedGradientDescentSolver::id(),
                    ProjectedGradientDescentSolver::ConvergenceTolOptionName(),
                    1e0);

  MathematicalProgramResult result =
      solver_.Solve(prog_, Vector1d(1.0), options);
  EXPECT_TRUE(result.is_success());
  auto x_value = result.GetSolution(x_);
  EXPECT_GT(x_value[0], 1e-1);

  // Default convergence tolerance yields a more accurate solution.
  result = solver_.Solve(prog_, Vector1d(1.0), {});
  x_value = result.GetSolution(x_);
  EXPECT_LE(x_value[0], 1e-11);
}

TEST_F(SimpleUnconstrainedQP, MaxIterations) {
  SolverOptions options;
  options.SetOption(ProjectedGradientDescentSolver::id(),
                    ProjectedGradientDescentSolver::MaxIterationsOptionName(),
                    1);

  MathematicalProgramResult result =
      solver_.Solve(prog_, Vector1d(1.0), options);
  EXPECT_FALSE(result.is_success());
  EXPECT_EQ(result.get_solution_result(), SolutionResult::kIterationLimit);
}

}  // namespace test
}  // namespace solvers
}  // namespace drake
