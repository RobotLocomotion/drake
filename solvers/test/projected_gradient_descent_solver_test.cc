#include "drake/solvers/projected_gradient_descent_solver.h"

#include <algorithm>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/solvers/equality_constrained_qp_solver.h"
#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/solve.h"
#include "drake/solvers/test/linear_program_examples.h"
#include "drake/solvers/test/mathematical_program_test_util.h"
#include "drake/solvers/test/optimization_examples.h"
#include "drake/solvers/test/quadratic_program_examples.h"
#include "drake/solvers/test/second_order_cone_program_examples.h"

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

GTEST_TEST(ProjectedGradientDescentSolver, NameTest) {
  EXPECT_EQ(ProjectedGradientDescentSolver::id().name(), "PGD");
}

GTEST_TEST(ProjectedGradientDescentSolverTest, QP) {
  // First program is unconstrained
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<2>();
  prog.AddCost(pow(x(0) - 1, 2) + pow(x(1) - 1, 2));

  const double kTol = 1e-4;

  ProjectedGradientDescentSolver solver;
  MathematicalProgramResult result = solver.Solve(prog, {}, {});
  EXPECT_TRUE(result.is_success());
  VectorXd expected_answer = Vector2d(1.0, 1.0);
  auto x_value = result.GetSolution(x);
  EXPECT_TRUE(CompareMatrices(expected_answer, x_value, kTol,
                              MatrixCompareType::absolute));
  EXPECT_NEAR(0.0, result.get_optimal_cost(), kTol);

  // Add a linear equality constraint.
  prog.AddConstraint(x(0) == 2.0);
  result = solver.Solve(prog, {}, {});
  EXPECT_TRUE(result.is_success());
  expected_answer = Vector2d(2.0, 1.0);
  x_value = result.GetSolution(x);
  EXPECT_TRUE(CompareMatrices(expected_answer, x_value, kTol,
                              MatrixCompareType::absolute));
  EXPECT_NEAR(1.0, result.get_optimal_cost(), kTol);

  // Add a linear inequality constraint.
  prog.AddConstraint(x(1) <= 0.0);
  result = solver.Solve(prog, {}, {});
  EXPECT_TRUE(result.is_success());
  expected_answer = Vector2d(2.0, 0.0);
  x_value = result.GetSolution(x);
  EXPECT_TRUE(CompareMatrices(expected_answer, x_value, kTol,
                              MatrixCompareType::absolute));
  EXPECT_NEAR(2.0, result.get_optimal_cost(), kTol);
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

  const double kTol = 1e-4;

  result = solver.Solve(prog, Vector1d(-1.0), {});
  EXPECT_TRUE(result.is_success());
  x_value = result.GetSolution(x);
  EXPECT_NEAR(x_value[0], -2.0, kTol);
  EXPECT_NEAR(result.get_optimal_cost(), 9.0, kTol);

  result = solver.Solve(prog, Vector1d(1.0), {});
  EXPECT_TRUE(result.is_success());
  x_value = result.GetSolution(x);
  EXPECT_NEAR(x_value[0], 2.0, kTol);
  EXPECT_NEAR(result.get_optimal_cost(), 1.0, kTol);
}

GTEST_TEST(ProjectedGradientDescentSolverTest, CustomProjectionFunction) {
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<1>();
  prog.AddCost(pow(x(0), 2));

  // Note that we don't add any constraints to the program, to ensure the custom
  // projection function is being used. We restrict 1 <= x <= 3, and minimize
  // x^2.
  auto custom_projection_function = [](const Vector1d& y, VectorXd* z) {
    *z = Vector1d(std::max(1.0, std::min(y[0], 3.0)));
    return true;
  };

  ProjectedGradientDescentSolver solver;
  solver.SetCustomProjectionFunction(custom_projection_function);

  const double kTol = 1e-4;

  // We deliberately give it an infeasible initial guess to make sure the custom
  // projection function is being called properly.
  MathematicalProgramResult result = solver.Solve(prog, Vector1d(4.0), {});
  EXPECT_TRUE(result.is_success());
  auto x_value = result.GetSolution(x);
  EXPECT_NEAR(x_value[0], 1.0, kTol);
  EXPECT_NEAR(result.get_optimal_cost(), 1.0, kTol);
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

  const double kTol = 1e-4;

  // We deliberately give it an infeasible initial guess to make sure the custom
  // projection function is being called properly.
  MathematicalProgramResult result = solver.Solve(prog, Vector1d(4.0), {});
  EXPECT_TRUE(result.is_success());
  auto x_value = result.GetSolution(x);
  EXPECT_NEAR(x_value[0], 1.0, kTol);
  EXPECT_NEAR(result.get_optimal_cost(), 1.0, kTol);
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

TEST_F(SimpleUnconstrainedQP, ConvergenceTol) {
  SolverOptions options;
  options.SetOption(ProjectedGradientDescentSolver::id(),
                    ProjectedGradientDescentSolver::ConvergenceTolOptionName(),
                    1e0);

  const double kTolLoose = 1e-1;
  const double kTolTight = 1e-11;

  MathematicalProgramResult result =
      solver_.Solve(prog_, Vector1d(1.0), options);
  EXPECT_TRUE(result.is_success());
  auto x_value = result.GetSolution(x_);
  EXPECT_GT(x_value[0], kTolLoose);

  // Default convergence tolerance yields a more accurate solution.
  result = solver_.Solve(prog_, Vector1d(1.0), {});
  x_value = result.GetSolution(x_);
  EXPECT_LE(x_value[0], kTolTight);

  // Check the error message if an invalid value is used.
  options.SetOption(ProjectedGradientDescentSolver::id(),
                    ProjectedGradientDescentSolver::ConvergenceTolOptionName(),
                    0.0);
  DRAKE_EXPECT_THROWS_MESSAGE(
      solver_.Solve(prog_, {}, options),
      ".*ConvergenceTol should be a non-negative number.*");
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

  // Check the error message if an invalid value is used.
  options.SetOption(ProjectedGradientDescentSolver::id(),
                    ProjectedGradientDescentSolver::MaxIterationsOptionName(),
                    0);
  DRAKE_EXPECT_THROWS_MESSAGE(solver_.Solve(prog_, {}, options),
                              ".*MaxIterations must be at least one.*");
}

TEST_F(SimpleUnconstrainedQP, BacktrackingC) {
  SolverOptions options;
  options.SetOption(ProjectedGradientDescentSolver::id(),
                    ProjectedGradientDescentSolver::BacktrackingCOptionName(),
                    0.5);
  EXPECT_NO_THROW(solver_.Solve(prog_, {}, options));

  options.SetOption(ProjectedGradientDescentSolver::id(),
                    ProjectedGradientDescentSolver::BacktrackingCOptionName(),
                    0.0);
  DRAKE_EXPECT_THROWS_MESSAGE(solver_.Solve(prog_, {}, options),
                              ".*BacktrackingC must be between 0 and 1.*");

  options.SetOption(ProjectedGradientDescentSolver::id(),
                    ProjectedGradientDescentSolver::BacktrackingCOptionName(),
                    1.0);
  DRAKE_EXPECT_THROWS_MESSAGE(solver_.Solve(prog_, {}, options),
                              ".*BacktrackingC must be between 0 and 1.*");
}

TEST_F(SimpleUnconstrainedQP, BacktrackingTau) {
  SolverOptions options;
  options.SetOption(ProjectedGradientDescentSolver::id(),
                    ProjectedGradientDescentSolver::BacktrackingTauOptionName(),
                    0.5);
  EXPECT_NO_THROW(solver_.Solve(prog_, {}, options));

  options.SetOption(ProjectedGradientDescentSolver::id(),
                    ProjectedGradientDescentSolver::BacktrackingTauOptionName(),
                    0.0);
  DRAKE_EXPECT_THROWS_MESSAGE(solver_.Solve(prog_, {}, options),
                              ".*BacktrackingTau must be between 0 and 1.*");

  options.SetOption(ProjectedGradientDescentSolver::id(),
                    ProjectedGradientDescentSolver::BacktrackingTauOptionName(),
                    1.0);
  DRAKE_EXPECT_THROWS_MESSAGE(solver_.Solve(prog_, {}, options),
                              ".*BacktrackingTau must be between 0 and 1.*");
}

TEST_F(SimpleUnconstrainedQP, BacktrackingAlpha0) {
  SolverOptions options;
  options.SetOption(
      ProjectedGradientDescentSolver::id(),
      ProjectedGradientDescentSolver::BacktrackingAlpha0OptionName(), 0.5);
  EXPECT_NO_THROW(solver_.Solve(prog_, {}, options));

  options.SetOption(
      ProjectedGradientDescentSolver::id(),
      ProjectedGradientDescentSolver::BacktrackingAlpha0OptionName(), 0.0);
  DRAKE_EXPECT_THROWS_MESSAGE(
      solver_.Solve(prog_, {}, options),
      ".*BacktrackingAlpha0 should be a non-negative number.*");
}

TEST_F(InfeasibleLinearProgramTest0, TestProjectedGradientDescentSolver) {
  prog_->SetInitialGuessForAllVariables(Eigen::Vector2d(1, 2));
  ProjectedGradientDescentSolver solver;
  auto result = solver.Solve(*prog_, {}, {});
  EXPECT_EQ(result.get_solution_result(),
            SolutionResult::kInfeasibleConstraints);
}

TEST_F(UnboundedLinearProgramTest0, TestProjectedGradientDescentSolver) {
  prog_->SetInitialGuessForAllVariables(Eigen::Vector2d::Zero());
  ProjectedGradientDescentSolver solver;
  auto result = solver.Solve(*prog_, {}, {});
  EXPECT_EQ(result.get_solution_result(), SolutionResult::kIterationLimit);
}

GTEST_TEST(ProjectedGradientDescentSolverTest, TestNonconvexQP) {
  ProjectedGradientDescentSolver solver;

  const double kConvergenceTol = 1e-7;
  SolverOptions options;
  options.SetOption(ProjectedGradientDescentSolver::id(),
                    ProjectedGradientDescentSolver::ConvergenceTolOptionName(),
                    kConvergenceTol);

  const double kTol = 3e-4;
  TestNonconvexQP(solver, false, kTol, &options);
}

GTEST_TEST(ProjectedGradientDescentSolverTest, TestL2NormCost) {
  ProjectedGradientDescentSolver solver;
  const double kTol = 1e-6;
  TestL2NormCost(solver, kTol);
}

}  // namespace test
}  // namespace solvers
}  // namespace drake
