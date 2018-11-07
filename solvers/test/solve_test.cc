#include "drake/solvers/solve.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/solvers/linear_system_solver.h"

namespace drake {
namespace solvers {
GTEST_TEST(SolveTest, LinearSystemSolverTest) {
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<2>();
  prog.AddLinearEqualityConstraint(Eigen::Matrix2d::Identity(),
                                   Eigen::Vector2d(1, 2), x);
  auto result = Solve(prog, {}, {});
  EXPECT_EQ(result.get_solution_result(), SolutionResult::kSolutionFound);
  EXPECT_TRUE(
      CompareMatrices(result.get_x_val(), Eigen::Vector2d(1, 2), 1E-12));
  EXPECT_EQ(result.get_optimal_cost(), 0);
  EXPECT_EQ(result.get_solver_id(), LinearSystemSolver::id());

  // Now add an inconsistent constraint
  prog.AddLinearEqualityConstraint(x(0) + x(1), 5);
  result = Solve(prog, {}, {});
  EXPECT_EQ(result.get_solution_result(),
            SolutionResult::kInfeasibleConstraints);
  EXPECT_EQ(result.get_optimal_cost(),
            MathematicalProgram::kGlobalInfeasibleCost);
  EXPECT_EQ(result.get_solver_id(), LinearSystemSolver::id());
}

// TODO(hongkai.dai): add a test that will appropriately set the initial guess
// and the solver options. Probably for SNOPT.
}  // namespace solvers
}  // namespace drake
