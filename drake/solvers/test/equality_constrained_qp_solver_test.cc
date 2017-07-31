#include "drake/solvers/equality_constrained_qp_solver.h"

#include <gtest/gtest.h>

#include "drake/common/eigen_matrix_compare.h"
#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/test/mathematical_program_test_util.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using Eigen::Vector2d;
using Eigen::Matrix2d;

namespace drake {
namespace solvers {
namespace test {
//
// Test how an unconstrained QP is dispatched and solved:
//   - on the problem (x1 - 1)^2 + (x2 - 1)^2, with a min at
//     at (x1=1, x2=1).
//   - on the same problem plus the additional problem
//     (x2 - 3)^2 + (2*x3 - 4)^2, which, when combined
//     with the first problem, has min at (x1=1, x2=2, x3=2)
// The first case tests a single quadratic cost, and the
// second case tests multiple quadratic costs affecting
// different variable views. All fall under the
// umbrella of the Equality Constrained QP Solver.
GTEST_TEST(testMathematicalProgram, testUnconstrainedQPDispatch) {
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<2>();
  prog.AddCost(pow(x(0) - 1, 2) + pow(x(1) - 1, 2));

  prog.SetInitialGuessForAllVariables(Eigen::Vector2d::Zero());
  prog.Solve();

  VectorXd expected_answer(2);
  expected_answer << 1.0, 1.0;
  auto x_value = prog.GetSolution(x);
  EXPECT_TRUE(CompareMatrices(expected_answer, x_value, 1e-10,
                              MatrixCompareType::absolute));
  EXPECT_NEAR(0.0, prog.GetOptimalCost(), 1e-10);

  // There are no inequality constraints, and only quadratic costs,
  // so this should hold:
  CheckSolver(prog, EqualityConstrainedQPSolver::id());

  // Add one more variable and constrain a view into them.
  auto y = prog.NewContinuousVariables<1>("y");

  prog.AddCost(pow(x(1) - 3, 2) + pow(2 * y(0) - 4, 2));
  prog.SetInitialGuessForAllVariables(Eigen::Vector3d::Zero());
  prog.Solve();
  expected_answer.resize(3);
  expected_answer << 1.0, 2.0, 2.0;
  VectorXd actual_answer(3);
  x_value = prog.GetSolution(x);
  const auto& y_value = prog.GetSolution(y);
  actual_answer << x_value, y_value;
  EXPECT_TRUE(CompareMatrices(expected_answer, actual_answer, 1e-10,
                              MatrixCompareType::absolute))
            << "\tExpected: " << expected_answer.transpose()
            << "\tActual: " << actual_answer.transpose();
  EXPECT_NEAR(2.0, prog.GetOptimalCost(), 1e-10);

  // Problem still has only quadratic costs, so solver should be the same.
  CheckSolver(prog, EqualityConstrainedQPSolver::id());
}

// Test how an equality-constrained QP is dispatched
//   - on the problem (x1 - 1)^2 + (x2 - 1)^2, with a min at
//     at (x1=1, x2=1), constrained with (x1 + x2 = 1).
//     The resulting constrained min is at (x1=0.5, x2=0.5).
//   - on the same problem with an additional variable x3,
//     with (2*x1 - x3 = 0). Resulting solution should be
//     (x1=0.5, x2=0.5, x3=1.0)
GTEST_TEST(testMathematicalProgram, testLinearlyConstrainedQPDispatch) {
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables(2);

  prog.AddCost(pow(x(0) - 1, 2) + pow(x(1) - 1, 2));

  // x1 + x2 = 1
  prog.AddLinearConstraint(x(0) + x(1) == 1);

  prog.SetInitialGuessForAllVariables(Eigen::Vector2d::Zero());
  prog.Solve();

  VectorXd expected_answer(2);
  expected_answer << 0.5, 0.5;
  auto x_value = prog.GetSolution(x);
  EXPECT_TRUE(CompareMatrices(expected_answer, x_value, 1e-10,
                              MatrixCompareType::absolute));

  EXPECT_NEAR(0.5, prog.GetOptimalCost(), 1e-10);

  // This problem is now an Equality Constrained QP and should
  // use this solver:
  CheckSolver(prog, EqualityConstrainedQPSolver::id());

  // Add one more variable and constrain it in a different way
  auto y = prog.NewContinuousVariables(1);
  Vector2d constraint2(2);
  // 2*x1 - x3 = 0, so x3 should wind up as 1.0

  prog.AddLinearConstraint(2 * x(0) - y(0) == 0);
  prog.SetInitialGuessForAllVariables(Eigen::Vector3d::Zero());
  prog.Solve();
  expected_answer.resize(3);
  expected_answer << 0.5, 0.5, 1.0;
  VectorXd actual_answer(3);
  x_value = prog.GetSolution(x);
  auto y_value = prog.GetSolution(y);
  actual_answer << x_value, y_value;
  EXPECT_TRUE(CompareMatrices(expected_answer, actual_answer, 1e-10,
                              MatrixCompareType::absolute))
            << "\tExpected: " << expected_answer.transpose()
            << "\tActual: " << actual_answer.transpose();
  EXPECT_NEAR(0.5, prog.GetOptimalCost(), 1e-10);
}

GTEST_TEST(testMathematicalProgram, testNotStrictlyPositiveDefiniteHessianQP) {
  // Cost is x(0)² - x(0). The Hessian is positive semidefinite, but not
  // strictly positive definite, as it has an eigen value equal to 0.
  // The problem has infinitely many optimal solutions, as (0.5, ANYTHING), and
  // a unique optimal cost -0.25.
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<2>("x");
  prog.AddCost((x(0) - 1) * x(0));
  EqualityConstrainedQPSolver equality_qp_solver;
  auto result = equality_qp_solver.Solve(prog);
  EXPECT_EQ(result, SolutionResult::kSolutionFound);
  EXPECT_NEAR(prog.GetSolution(x(0)), 0.5, 1E-10);
  EXPECT_NEAR(prog.GetOptimalCost(), -0.25, 1E-10);
}

GTEST_TEST(testMathematicalProgram, testNonPositiveSemidefiniteHessianQP) {
  // Cost is x(0)² - x(1)². The Hessian has a negative eigen value, the problem
  // is unbounded.
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<2>("x");
  prog.AddCost(x(0) * x(0) -x(1) * x(1));
  EqualityConstrainedQPSolver equality_qp_solver;
  auto result = equality_qp_solver.Solve(prog);
  EXPECT_EQ(result, SolutionResult::kUnbounded);
}

GTEST_TEST(testMathematicalProgram, testUnboundedQP) {
  // Cost is x(0)² - 2 * x(1). The Hessian is positive semidefinite, but not
  // strictly positive definite, as it has an eigen value equal to 0.
  // The problem is unbounded, since x(1) can be as large as possible.
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<2>("x");
  prog.AddCost(x(0) * x(0) - 2 * x(1));
  EqualityConstrainedQPSolver equality_qp_solver;
  auto result = equality_qp_solver.Solve(prog);
  EXPECT_EQ(result, SolutionResult::kUnbounded);
}

GTEST_TEST(testMathematicalProgram, testNegativeDefiniteHessianQP) {
  // Cost is -x(0)² - 2 * x(1). The Hessian is negative semidefinite.
  // The problem is unbounded.
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<2>("x");
  prog.AddCost(-x(0) * x(0) - 2 * x(1));
  EqualityConstrainedQPSolver equality_qp_solver;
  auto result = equality_qp_solver.Solve(prog);
  EXPECT_EQ(result, SolutionResult::kUnbounded);
}
}  // namespace test
}  // namespace solvers
}  // namespace drake
