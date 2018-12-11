#include "drake/solvers/equality_constrained_qp_solver.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"
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
GTEST_TEST(testEqualityConstrainedQPSolver, testUnconstrainedQPDispatch) {
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<2>();
  prog.AddCost(pow(x(0) - 1, 2) + pow(x(1) - 1, 2));

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
GTEST_TEST(testEqualityConstrainedQPSolver, testLinearlyConstrainedQPDispatch) {
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

GTEST_TEST(testEqualityConstrainedQPSolver,
           testNotStrictlyPositiveDefiniteHessianQP) {
  // Cost is x(0)² - x(0). The Hessian is positive semidefinite, but not
  // strictly positive definite, as it has an eigen value equal to 0.
  // The problem has infinitely many optimal solutions, as (0.5, ANYTHING), and
  // a unique optimal cost -0.25.
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<2>("x");
  prog.AddCost((x(0) - 1) * x(0));
  EqualityConstrainedQPSolver equality_qp_solver;
  MathematicalProgramResult result;
  equality_qp_solver.Solve(prog, {}, {}, &result);
  EXPECT_EQ(result.get_solution_result(), SolutionResult::kSolutionFound);
  EXPECT_NEAR(prog.GetSolution(x(0), result), 0.5, 1E-10);
  EXPECT_NEAR(result.get_optimal_cost(), -0.25, 1E-10);
}

GTEST_TEST(testEqualityConstrainedQPSolver,
           testNonPositiveSemidefiniteHessianQP) {
  // Cost is x(0)² - x(1)². The Hessian has a negative eigen value, the problem
  // is unbounded.
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<2>("x");
  prog.AddCost(x(0) * x(0) - x(1) * x(1));
  EqualityConstrainedQPSolver equality_qp_solver;
  auto result = equality_qp_solver.Solve(prog);
  EXPECT_EQ(result, SolutionResult::kUnbounded);
  EXPECT_EQ(prog.GetOptimalCost(), MathematicalProgram::kUnboundedCost);
}

GTEST_TEST(testEqualityConstrainedQPSolver, testUnboundedQP) {
  // Cost is x(0)² - 2 * x(1). The Hessian is positive semidefinite, but not
  // strictly positive definite, as it has an eigen value equal to 0.
  // The problem is unbounded, since x(1) can be as large as possible.
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<2>("x");
  prog.AddCost(x(0) * x(0) - 2 * x(1));
  EqualityConstrainedQPSolver equality_qp_solver;
  auto result = equality_qp_solver.Solve(prog);
  EXPECT_EQ(result, SolutionResult::kUnbounded);
  EXPECT_EQ(prog.GetOptimalCost(), MathematicalProgram::kUnboundedCost);
}

GTEST_TEST(testEqualityConstrainedQPSolver, testNegativeDefiniteHessianQP) {
  // Cost is -x(0)² - 2 * x(1). The Hessian is negative semidefinite.
  // The problem is unbounded.
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<2>("x");
  prog.AddCost(-x(0) * x(0) - 2 * x(1));
  EqualityConstrainedQPSolver equality_qp_solver;
  auto result = equality_qp_solver.Solve(prog);
  EXPECT_EQ(result, SolutionResult::kUnbounded);
  EXPECT_EQ(prog.GetOptimalCost(), MathematicalProgram::kUnboundedCost);
}

// Test a QP with positive definite Hessian, but infeasible constraint.
// min x(0)² + 2 * x(1)² + x(0) * x(1)
// s.t  x(0) + 2 * x(1) = 1
//      x(0) - x(1) = 3
//      2 * x(0) + x(1) = 2
GTEST_TEST(testEqualityConstrainedQPSolver,
           testPositiveHessianInfeasibleConstraint) {
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<2>("x");
  prog.AddCost(x(0) * x(0) + 2 * x(1) * x(1) + x(0) * x(1));
  prog.AddLinearConstraint(x(0) + 2 * x(1) == 1 && x(0) - x(1) == 3 &&
                           2 * x(0) + x(1) == 2);
  EqualityConstrainedQPSolver equality_qp_solver;
  auto result = equality_qp_solver.Solve(prog);
  EXPECT_EQ(result, SolutionResult::kInfeasibleConstraints);
  EXPECT_EQ(prog.GetOptimalCost(), MathematicalProgram::kGlobalInfeasibleCost);
}

// Test a QP with indefinite Hessian, but unique minimum.
// min x(0)² - x(1)²
// s.t x(1) = 1
GTEST_TEST(testEqualityConstrainedQPSolver, testIndefiniteHessian) {
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<2>("x");
  prog.AddCost(x(0) * x(0) - x(1) * x(1));
  prog.AddLinearConstraint(x(1) == 1);
  EqualityConstrainedQPSolver equality_qp_solver;
  auto result = equality_qp_solver.Solve(prog);
  EXPECT_EQ(result, SolutionResult::kSolutionFound);
  EXPECT_TRUE(CompareMatrices(prog.GetSolution(x), Eigen::Vector2d(0, 1), 1E-12,
                              MatrixCompareType::absolute));
  EXPECT_NEAR(prog.GetOptimalCost(), -1, 1E-12);
}

// Test a QP with positive semidefinite Hessian, but unbounded objective.
// min x(0)² - 2 * x(1)
// s.t x(0) = 1
GTEST_TEST(testEqualityConstrainedQPSolver, testPSDhessianUnbounded) {
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<2>("x");
  prog.AddCost(x(0) * x(0) - 2 * x(1));
  prog.AddLinearConstraint(x(0) == 1);
  EqualityConstrainedQPSolver equality_qp_solver;
  auto result = equality_qp_solver.Solve(prog);
  EXPECT_EQ(result, SolutionResult::kUnbounded);
  EXPECT_EQ(prog.GetOptimalCost(), MathematicalProgram::kUnboundedCost);
}

// Test a QP with negative definite Hessian, but with a unique optimum.
// min -x(0)² - x(1)²
// s.t x(0) + x(1) = 2
//     x(0) - x(1) = 3
GTEST_TEST(testEqualityConstrainedQPSolver, testNegativeHessianUniqueOptimum) {
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<2>("x");
  prog.AddCost(-x(0) * x(0) - x(1) * x(1));
  prog.AddLinearConstraint(x(0) + x(1) == 2 && x(0) - x(1) == 3);
  EqualityConstrainedQPSolver equality_qp_solver;
  auto result = equality_qp_solver.Solve(prog);
  EXPECT_EQ(result, SolutionResult::kSolutionFound);
  EXPECT_TRUE(CompareMatrices(prog.GetSolution(x), Eigen::Vector2d(2.5, -0.5),
                              1E-12, MatrixCompareType::absolute));
  EXPECT_NEAR(prog.GetOptimalCost(), -6.5, 1E-12);
}

// Test a QP with negative definite Hessian, and an unbounded objective.
// min -x(0)² - x(1)²
// s.t x(0) + x(1) == 1
GTEST_TEST(testEqualityConstrainedQPSolver, testNegativeHessianUnbounded) {
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<2>("x");
  prog.AddCost(-x(0) * x(0) - x(1) * x(1));
  prog.AddLinearConstraint(x(0) + x(1) == 1);
  EqualityConstrainedQPSolver equality_qp_solver;
  auto result = equality_qp_solver.Solve(prog);
  EXPECT_EQ(result, SolutionResult::kUnbounded);
  EXPECT_EQ(prog.GetOptimalCost(), MathematicalProgram::kUnboundedCost);
}

// Test a QP with positive semidefinite Hessian (not strictly positive
// definite), and a unique minimum.
// min x(0)² + 2 * x(0) + 3 * x(1)
// s.t x(0) + 2 * x(1) == 1
GTEST_TEST(testEqualityConstrainedQPSolver, testPSDHessianUniqueOptimal) {
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<2>("x");
  prog.AddCost(x(0) * x(0) + 2 * x(0) + 3 * x(1));
  prog.AddLinearConstraint(x(0) + 2 * x(1) == 1);
  EqualityConstrainedQPSolver equality_qp_solver;
  auto result = equality_qp_solver.Solve(prog);
  EXPECT_EQ(result, SolutionResult::kSolutionFound);
  EXPECT_TRUE(CompareMatrices(prog.GetSolution(x),
                              Eigen::Vector2d(-1.0 / 4, 5.0 / 8), 1E-12,
                              MatrixCompareType::absolute));
  EXPECT_NEAR(prog.GetOptimalCost(), 23.0 / 16.0, 1E-12);
}

// Test a QP with indefinite Hessian and infeasible constraints
// min x(0)² - 2 * x(1)²
// s.t x(0) + 2 * x(1) = 1
//     -x(0) + 3 * x(1) = 2
//     2 * x(0) - 3 * x(1) = 3
GTEST_TEST(testEqualityConstrainedQPSolver, testIndefiniteHessianInfeasible) {
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<2>("x");
  prog.AddCost(x(0) * x(0) - 2 * x(1) * x(1));
  prog.AddLinearConstraint(x(0) + 2 * x(1) == 1 && -x(0) + 3 * x(1) == 2 &&
                           2 * x(0) - 3 * x(1) == 3);
  EqualityConstrainedQPSolver equality_qp_solver;
  auto result = equality_qp_solver.Solve(prog);
  EXPECT_EQ(result, SolutionResult::kInfeasibleConstraints);
  EXPECT_EQ(prog.GetOptimalCost(), MathematicalProgram::kGlobalInfeasibleCost);
}

// Test changing the feasibility tolerance.
// For a problem
// min x(0)² + 2 * x(1)²
// s.t x(0) + 2 * x(1) = 1
//     x(0) - x(1) = -2
//     x(0) + x(1) = 1E-6
// when the feasibility tolerance is 1E-7, the problem is infeasible.
// when we increase the feasibility tolerance, the problem is feasible.
GTEST_TEST(testEqualityConstrainedQPSolver, testFeasibilityTolerance) {
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<2>("x");
  prog.AddCost(x(0) * x(0) + 2 * x(1) * x(1));
  prog.AddLinearConstraint(x(0) + 2 * x(1) == 1 && x(0) - x(1) == -2 &&
                           x(0) + x(1) == 1E-6);
  EqualityConstrainedQPSolver equality_qp_solver;
  prog.SetSolverOption(EqualityConstrainedQPSolver::id(), "FeasibilityTol",
                       1E-7);
  auto result = equality_qp_solver.Solve(prog);
  EXPECT_EQ(result, SolutionResult::kInfeasibleConstraints);
  EXPECT_EQ(prog.GetOptimalCost(), MathematicalProgram::kGlobalInfeasibleCost);

  // Now increase the feasibility tolerance.
  double tol = 1E-6;
  prog.SetSolverOption(EqualityConstrainedQPSolver::id(),
                       EqualityConstrainedQPSolver::FeasibilityTolOptionName(),
                       tol);
  result = equality_qp_solver.Solve(prog);
  EXPECT_EQ(result, SolutionResult::kSolutionFound);
  const Eigen::Vector2d x_val = prog.GetSolution(x);
  const Eigen::Vector3d cnstr_val(x_val(0) + 2 * x_val(1), x_val(0) - x_val(1),
                                  x_val(0) + x_val(1));
  EXPECT_TRUE(CompareMatrices(cnstr_val, Eigen::Vector3d(1, -2, 1E-6), tol,
                              MatrixCompareType::absolute));
  EXPECT_NEAR(prog.GetOptimalCost(), 3, 1E-6);

  // Now solve with a low feasibility tolerance again by passing the option in
  // the Solver function. The result should be infeasible.
  MathematicalProgramResult math_prog_result;
  SolverOptions solver_options;
  // The input solver option (1E-7) in `Solve` function takes priority over the
  // option stored in the prog (1E-6).
  solver_options.SetOption(
      EqualityConstrainedQPSolver::id(),
      EqualityConstrainedQPSolver::FeasibilityTolOptionName(), 0.1 * tol);
  equality_qp_solver.Solve(prog, {}, solver_options, &math_prog_result);
  EXPECT_EQ(math_prog_result.get_solution_result(),
            SolutionResult::kInfeasibleConstraints);
}

// min x'*x + x0 + x1 + 1
// s.t. x0 = x1.
// (aka min 2x^2 + 2x + 1)
// The solution is x0 = x1 = -.5, with optimal value .5.
GTEST_TEST(testEqualityConstrainedQPSolver, testLinearCost) {
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<2>("x");

  prog.AddQuadraticCost(x.transpose() * x);
  prog.AddLinearCost(x(0) + x(1) + 1);

  EXPECT_EQ(prog.Solve(), SolutionResult::kSolutionFound);

  EXPECT_TRUE(
      CompareMatrices(prog.GetSolution(x), Eigen::Vector2d(-.5, -.5), 1e-6));
  EXPECT_EQ(prog.GetOptimalCost(), .5);
}

class EqualityConstrainedQPSolverTest : public ::testing::Test {
 public:
  EqualityConstrainedQPSolverTest()
      : prog_{},
        x_{prog_.NewContinuousVariables<2>()},
        solver_{},
        result_{},
        solver_options_{} {
    prog_.AddLinearEqualityConstraint(x_(0) + x_(1), 1);
    prog_.AddQuadraticCost(x_(0) * x_(0) + x_(1) * x_(1));
  }

 protected:
  MathematicalProgram prog_;
  VectorDecisionVariable<2> x_;
  EqualityConstrainedQPSolver solver_;
  MathematicalProgramResult result_;
  SolverOptions solver_options_;
};

TEST_F(EqualityConstrainedQPSolverTest, WrongSolverOptions1) {
  solver_options_.SetOption(solver_.solver_id(), "Foo", 0.1);
  DRAKE_EXPECT_THROWS_MESSAGE_IF_ARMED(
      solver_.Solve(prog_, {}, solver_options_, &result_),
      std::invalid_argument,
      "Foo is not allowed in the SolverOptions for Equality constrained QP.");
}

TEST_F(EqualityConstrainedQPSolverTest, WrongSolverOptions2) {
  solver_options_.SetOption(solver_.solver_id(), "FeasibilityTol", -0.1);
  DRAKE_EXPECT_THROWS_MESSAGE(
      solver_.Solve(prog_, {}, solver_options_, &result_),
      std::invalid_argument, "FeasibilityTol should be a non-negative number.");
}

}  // namespace test
}  // namespace solvers
}  // namespace drake
