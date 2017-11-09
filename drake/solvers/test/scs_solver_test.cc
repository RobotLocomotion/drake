#include "drake/solvers/scs_solver.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/test/linear_program_examples.h"
#include "drake/solvers/test/mathematical_program_test_util.h"
#include "drake/solvers/test/quadratic_program_examples.h"

namespace drake {
namespace solvers {
namespace test {
GTEST_TEST(LinearProgramTest, Test0) {
  // Test a linear program with only equality constraint.
  // min x(0) + 2 * x(1)
  // s.t x(0) + x(1) = 2
  // The problem is unbounded.
  MathematicalProgram prog;
  const auto x = prog.NewContinuousVariables<2>("x");
  prog.AddLinearCost(x(0) + 2 * x(1));
  prog.AddLinearConstraint(x(0) + x(1) == 2);
  ScsSolver solver;
  if (solver.available()) {
    SolutionResult sol_result = solver.Solve(prog);
    EXPECT_EQ(sol_result, SolutionResult::kUnbounded);
  }

  // Now add the constraint x(1) <= 1. The problem should still be unbounded.
  prog.AddBoundingBoxConstraint(-std::numeric_limits<double>::infinity(), 1, x(1));
  if (solver.available()) {
    SolutionResult sol_result = solver.Solve(prog);
    EXPECT_EQ(sol_result, SolutionResult::kUnbounded);
  }

  // The accuracy of SCS is not high, so we choose 1E-3 here, same as the
  // epsilon defined in SCS.
  const double tol{1E-3};
  // Now add the constraint x(0) <= 5. The problem should be feasible. The
  // optimal cost is -1, with x = (5, -3)
  prog.AddBoundingBoxConstraint(-std::numeric_limits<double>::infinity(), 5, x(0));
  if (solver.available()) {
    SolutionResult sol_result = solver.Solve(prog);
    EXPECT_EQ(sol_result, SolutionResult::kSolutionFound);
    EXPECT_NEAR(prog.GetOptimalCost(), -1, tol);
    const Eigen::Vector2d x_expected(5, -3);
    EXPECT_TRUE(CompareMatrices(prog.GetSolution(x), x_expected, tol, MatrixCompareType::absolute));
  }

  // Now change the cost to 3x(0) - x(1) + 5, and add the constraint 2 <= x(0)
  // The problem is
  // min 3x(0) - x(1) + 5
  // s.t x(0) + x(1) = 2
  //     2 <= x(0) <= 5
  //          x(1) <= 1
  // The optimal cost is 11, the optimal solution is x = (2, 0)
  prog.AddLinearCost(2 * x(0) - 3 * x(1) + 5);
  prog.AddBoundingBoxConstraint(2, 6, x(0));
  if (solver.available()) {
    SolutionResult sol_result = solver.Solve(prog);
    EXPECT_EQ(sol_result, SolutionResult::kSolutionFound);
    EXPECT_NEAR(prog.GetOptimalCost(), 11, tol);
    const Eigen::Vector2d x_expected(2, 0);
    EXPECT_TRUE(CompareMatrices(prog.GetSolution(x), x_expected, tol, MatrixCompareType::absolute));
  }
}

TEST_P(LinearProgramTest, TestLP) {
  ScsSolver solver;
  prob()->RunProblem(&solver);
}

INSTANTIATE_TEST_CASE_P(
    SCSTest, LinearProgramTest,
    ::testing::Combine(::testing::ValuesIn(linear_cost_form()),
                       ::testing::ValuesIn(linear_constraint_form()),
                       ::testing::ValuesIn(linear_problems())));
}  // namespace test
}  // namespace solvers
}  // namespace drake
