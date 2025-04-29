#include "drake/solvers/projected_gradient_descent_solver.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/solve.h"

namespace drake {
namespace solvers {
namespace test {

using Eigen::VectorXd;

GTEST_TEST(ProjectedGradientDescentSolverTest, UnconstrainedQP) {
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<2>();
  prog.AddCost(pow(x(0) - 1, 2) + pow(x(1) - 1, 2));

  ProjectedGradientDescentSolver solver;
  MathematicalProgramResult result = solver.Solve(prog, {}, {});

  VectorXd expected_answer(2);
  expected_answer << 1.0, 1.0;
  auto x_value = result.GetSolution(x);
  EXPECT_TRUE(CompareMatrices(expected_answer, x_value, 1e-4,
                              MatrixCompareType::absolute));
  EXPECT_NEAR(0.0, result.get_optimal_cost(), 1e-4);
}

GTEST_TEST(ProjectedGradientDescentSolverTest, EqualityConstrainedQP) {
  //
}

GTEST_TEST(ProjectedGradientDescentSolverTest, GeneralQP) {
  //
}

}  // namespace test
}  // namespace solvers
}  // namespace drake
