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
//     (2*x2 - 5)^2 + (2*x3 - 2)^2, which, when combined
//     with the first problem, has min at (x1=1, x2=2, x3=1)
// The first case tests a single quadratic cost, and the
// second case tests multiple quadratic costs affecting
// different variable views. All fall under the
// umbrella of the Equality Constrained QP Solver.
GTEST_TEST(testMathematicalProgram, testUnconstrainedQPDispatch) {
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<2>();
  Matrix2d Q = Matrix2d::Identity();
  Vector2d c(-1, -1);

  prog.AddQuadraticCost(Q, c, x);

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
  Q << 2.0, 0.0, 0.0, 2.0;
  c << -5.0, -2.0;
  VariableRefList vars;
  vars.push_back(x.segment<1>(1));
  vars.push_back(y);

  prog.AddQuadraticCost(Q, c, vars);
  prog.SetInitialGuessForAllVariables(Eigen::Vector3d::Zero());
  prog.Solve();
  expected_answer.resize(3);
  expected_answer << 1.0, 2.0, 1.0;
  VectorXd actual_answer(3);
  x_value = prog.GetSolution(x);
  const auto& y_value = prog.GetSolution(y);
  actual_answer << x_value, y_value;
  EXPECT_TRUE(CompareMatrices(expected_answer, actual_answer, 1e-10,
                              MatrixCompareType::absolute))
            << "\tExpected: " << expected_answer.transpose()
            << "\tActual: " << actual_answer.transpose();
  EXPECT_NEAR(0.0, prog.GetOptimalCost(), 1e-10);

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
  MatrixXd Q(2, 2);
  Q << 1, 0.0, 0.0, 1.0;
  VectorXd c(2);
  c << -1.0, -1.0;

  prog.AddQuadraticCost(Q, c, x);

  VectorXd constraint1(2);
  // x1 + x2 = 1
  constraint1 << 1, 1;
  prog.AddLinearEqualityConstraint(constraint1.transpose(), 1.0, x);

  prog.SetInitialGuessForAllVariables(Eigen::Vector2d::Zero());
  prog.Solve();

  VectorXd expected_answer(2);
  expected_answer << 0.5, 0.5;
  auto x_value = prog.GetSolution(x);
  EXPECT_TRUE(CompareMatrices(expected_answer, x_value, 1e-10,
                              MatrixCompareType::absolute));

  EXPECT_NEAR(-0.25, prog.GetOptimalCost(), 1e-10);

  // This problem is now an Equality Constrained QP and should
  // use this solver:
  CheckSolver(prog, EqualityConstrainedQPSolver::id());

  // Add one more variable and constrain it in a different way
  auto y = prog.NewContinuousVariables(1);
  Vector2d constraint2(2);
  constraint2 << 2., -1.;
  // 2*x1 - x3 = 0, so x3 should wind up as 1.0
  VariableRefList vars;
  vars.push_back(x.segment(0, 1));
  vars.push_back(y);

  prog.AddLinearEqualityConstraint(constraint2.transpose(), 0.0, vars);
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
  EXPECT_NEAR(-0.25, prog.GetOptimalCost(), 1e-10);
}

GTEST_TEST(testMathematicalProgram, testNotStrictlyPositiveDefiniteHessianQP) {
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<2>("x");
  prog.AddCost(x(0) * x(0));
  EqualityConstrainedQPSolver equality_qp_solver;
  auto result = equality_qp_solver.Solve(prog);
  EXPECT_EQ(result, SolutionResult::kInvalidInput);
}
}  // namespace test
}  // namespace solvers
}  // namespace drake
