#include "drake/solvers/osqp_solver.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/test/quadratic_program_examples.h"

namespace drake {
namespace solvers {
namespace test {
GTEST_TEST(QPtest, TestUnconstrainedQP) {
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<3>("x");
  prog.AddQuadraticCost(x(0) * x(0));

  OsqpSolver solver;
  if (solver.available()) {
    const SolutionResult result = solver.Solve(prog);
    EXPECT_EQ(result, SolutionResult::kSolutionFound);
    const double tol = 1E-10;
    EXPECT_NEAR(prog.GetSolution(x(0)), 0, tol);
    EXPECT_NEAR(prog.GetOptimalCost(), 0, tol);
  }

  // Add additional quadratic costs
  prog.AddQuadraticCost((x(1) + x(2) - 2) * (x(1) + x(2) - 2));
  if (solver.available()) {
    const SolutionResult result = solver.Solve(prog);
    EXPECT_EQ(result, SolutionResult::kSolutionFound);
    const double tol = 1E-10;
    EXPECT_NEAR(prog.GetSolution(x(0)), 0, tol);
    EXPECT_NEAR(prog.GetSolution(x(1)) + prog.GetSolution(x(2)), 2, tol);
    EXPECT_NEAR(prog.GetOptimalCost(), 0, tol);
  }

  // Add linear costs.
  prog.AddLinearCost(4 * x(0) + 5);
  // Now the cost is (x₀ + 2)² + (x₁ + x₂-2)² + 1
  if (solver.available()) {
    const SolutionResult result = solver.Solve(prog);
    EXPECT_EQ(result, SolutionResult::kSolutionFound);
    const double tol = 1E-10;
    EXPECT_NEAR(prog.GetSolution(x(0)), -2, tol);
    EXPECT_NEAR(prog.GetSolution(x(1)) + prog.GetSolution(x(2)), 2, tol);
    EXPECT_NEAR(prog.GetOptimalCost(), 1, tol);
  }
}

TEST_P(QuadraticProgramTest, TestQP) {
  OsqpSolver solver;
  prob()->RunProblem(&solver);
}

INSTANTIATE_TEST_CASE_P(
    OsqpTest, QuadraticProgramTest,
    ::testing::Combine(::testing::ValuesIn(quadratic_cost_form()),
                       ::testing::ValuesIn(linear_constraint_form()),
                       ::testing::ValuesIn(quadratic_problems())));

GTEST_TEST(QPtest, TestUnitBallExample) {
  OsqpSolver solver;
  if (solver.available()) {
    TestQPonUnitBallExample(solver);
  }
}

GTEST_TEST(QPtest, TestUnbounded) {
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<3>();

  prog.AddQuadraticCost(x(0) * x(0) + x(1));

  OsqpSolver solver;
  // The program is unbounded.
  if (solver.available()) {
    const SolutionResult result = solver.Solve(prog);
    EXPECT_EQ(result, SolutionResult::kDualInfeasible);
  }

  // Add a constraint
  prog.AddLinearConstraint(x(0) + 2 * x(2) == 2);
  prog.AddLinearConstraint(x(0) >= 0);
  if (solver.available()) {
    const SolutionResult result = solver.Solve(prog);
    EXPECT_EQ(result, SolutionResult::kDualInfeasible);
  }
}

GTEST_TEST(QPtest, TestInfeasible) {
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<2>();

  prog.AddQuadraticCost(x(0) * x(0) + 2 * x(1) * x(1));
  prog.AddLinearConstraint(x(0) + 2 * x(1) == 2);
  prog.AddLinearConstraint(x(0) >= 1);
  prog.AddLinearConstraint(x(1) >= 2);

  OsqpSolver solver;
  // The program is infeasible.
  if (solver.available()) {
    const SolutionResult result = solver.Solve(prog);
    EXPECT_EQ(result, SolutionResult::kInfeasibleConstraints);
    EXPECT_EQ(prog.GetOptimalCost(),
              MathematicalProgram::kGlobalInfeasibleCost);
  }
}

GTEST_TEST(LPtest, TestUnbounded) {
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<2>();

  prog.AddLinearCost(x(0) + x(1));

  // This problem is unbounded.
  OsqpSolver solver;
  if (solver.available()) {
    const SolutionResult result = solver.Solve(prog);
    EXPECT_EQ(result, SolutionResult::kDualInfeasible);
  }

  // Add some constraint, the program is still unbounded.
  prog.AddLinearConstraint(x(0) >= 1);
  prog.AddLinearConstraint(x(1) <= 2);
  if (solver.available()) {
    const SolutionResult result = solver.Solve(prog);
    EXPECT_EQ(result, SolutionResult::kDualInfeasible);
  }
}
}  // namespace test
}  // namespace solvers
}  // namespace drake
