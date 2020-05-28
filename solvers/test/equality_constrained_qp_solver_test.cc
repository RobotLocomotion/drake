#include "drake/solvers/equality_constrained_qp_solver.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/solve.h"
#include "drake/solvers/test/mathematical_program_test_util.h"
#include "drake/solvers/test/quadratic_program_examples.h"

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

  MathematicalProgramResult result = Solve(prog);

  VectorXd expected_answer(2);
  expected_answer << 1.0, 1.0;
  auto x_value = result.GetSolution(x);
  EXPECT_TRUE(CompareMatrices(expected_answer, x_value, 1e-10,
                              MatrixCompareType::absolute));
  EXPECT_NEAR(0.0, result.get_optimal_cost(), 1e-10);

  // There are no inequality constraints, and only quadratic costs,
  // so this should hold:
  EXPECT_EQ(result.get_solver_id(), EqualityConstrainedQPSolver::id());

  // Add one more variable and constrain a view into them.
  auto y = prog.NewContinuousVariables<1>("y");

  prog.AddCost(pow(x(1) - 3, 2) + pow(2 * y(0) - 4, 2));
  prog.SetInitialGuessForAllVariables(Eigen::Vector3d::Zero());
  result = Solve(prog);
  expected_answer.resize(3);
  expected_answer << 1.0, 2.0, 2.0;
  VectorXd actual_answer(3);
  x_value = result.GetSolution(x);
  const auto& y_value = result.GetSolution(y);
  actual_answer << x_value, y_value;
  EXPECT_TRUE(CompareMatrices(expected_answer, actual_answer, 1e-10,
                              MatrixCompareType::absolute))
      << "\tExpected: " << expected_answer.transpose()
      << "\tActual: " << actual_answer.transpose();
  EXPECT_NEAR(2.0, result.get_optimal_cost(), 1e-10);

  // Problem still has only quadratic costs, so solver should be the same.
  EXPECT_EQ(result.get_solver_id(), EqualityConstrainedQPSolver::id());
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
  auto constraint1 = prog.AddLinearConstraint(x(0) + x(1) == 1);

  prog.SetInitialGuessForAllVariables(Eigen::Vector2d::Zero());

  MathematicalProgramResult result = Solve(prog);

  VectorXd expected_answer(2);
  expected_answer << 0.5, 0.5;
  auto x_value = result.GetSolution(x);
  EXPECT_TRUE(CompareMatrices(expected_answer, x_value, 1e-10,
                              MatrixCompareType::absolute));

  EXPECT_NEAR(0.5, result.get_optimal_cost(), 1e-10);
  EXPECT_TRUE(CompareMatrices(result.GetDualSolution(constraint1), Vector1d(-1),
                              1e-14));

  // This problem is now an Equality Constrained QP and should
  // use this solver:
  EXPECT_EQ(result.get_solver_id(), EqualityConstrainedQPSolver::id());

  // Add one more variable and constrain it in a different way
  auto y = prog.NewContinuousVariables(1);
  // 2*x1 - y = 0, so y should wind up as 1.0
  auto constraint2 = prog.AddLinearConstraint(2 * x(0) - y(0) == 0);
  prog.SetInitialGuessForAllVariables(Eigen::Vector3d::Zero());
  result = Solve(prog, Eigen::Vector3d::Zero());
  expected_answer.resize(3);
  expected_answer << 0.5, 0.5, 1.0;
  VectorXd actual_answer(3);
  x_value = result.GetSolution(x);
  auto y_value = result.GetSolution(y);
  actual_answer << x_value, y_value;
  EXPECT_TRUE(CompareMatrices(expected_answer, actual_answer, 1e-10,
                              MatrixCompareType::absolute))
      << "\tExpected: " << expected_answer.transpose()
      << "\tActual: " << actual_answer.transpose();
  EXPECT_NEAR(0.5, result.get_optimal_cost(), 1e-10);
  EXPECT_TRUE(CompareMatrices(result.GetDualSolution(constraint1), Vector1d(-1),
                              1e-14));
  EXPECT_TRUE(CompareMatrices(result.GetDualSolution(constraint2), Vector1d(0.),
                              1e-14));
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
  EqualityConstrainedQPSolver solver;
  auto result = solver.Solve(prog, {}, {});
  EXPECT_TRUE(result.is_success());
  EXPECT_NEAR(result.GetSolution(x(0)), 0.5, 1E-10);
  EXPECT_NEAR(result.get_optimal_cost(), -0.25, 1E-10);
}

GTEST_TEST(testEqualityConstrainedQPSolver,
           testNonPositiveSemidefiniteHessianQP) {
  // Cost is x(0)² - x(1)². The Hessian has a negative eigen value, the problem
  // is unbounded.
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<2>("x");
  prog.AddCost(x(0) * x(0) - x(1) * x(1));
  EqualityConstrainedQPSolver solver;
  auto result = solver.Solve(prog, {}, {});
  EXPECT_EQ(result.get_solution_result(), SolutionResult::kUnbounded);
  EXPECT_EQ(result.get_optimal_cost(), MathematicalProgram::kUnboundedCost);
}

GTEST_TEST(testEqualityConstrainedQPSolver, testUnboundedQP) {
  // Cost is x(0)² - 2 * x(1). The Hessian is positive semidefinite, but not
  // strictly positive definite, as it has an eigen value equal to 0.
  // The problem is unbounded, since x(1) can be as large as possible.
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<2>("x");
  prog.AddCost(x(0) * x(0) - 2 * x(1));
  EqualityConstrainedQPSolver solver;
  auto result = solver.Solve(prog, {}, {});
  EXPECT_EQ(result.get_solution_result(), SolutionResult::kUnbounded);
  EXPECT_EQ(result.get_optimal_cost(), MathematicalProgram::kUnboundedCost);
}

GTEST_TEST(testEqualityConstrainedQPSolver, testNegativeDefiniteHessianQP) {
  // Cost is -x(0)² - 2 * x(1). The Hessian is negative semidefinite.
  // The problem is unbounded.
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<2>("x");
  prog.AddCost(-x(0) * x(0) - 2 * x(1));
  EqualityConstrainedQPSolver solver;
  auto result = solver.Solve(prog, {}, {});
  EXPECT_EQ(result.get_solution_result(), SolutionResult::kUnbounded);
  EXPECT_EQ(result.get_optimal_cost(), MathematicalProgram::kUnboundedCost);
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
  EqualityConstrainedQPSolver solver;
  auto result = solver.Solve(prog, {}, {});
  EXPECT_EQ(result.get_solution_result(),
            SolutionResult::kInfeasibleConstraints);
  EXPECT_EQ(result.get_optimal_cost(),
            MathematicalProgram::kGlobalInfeasibleCost);
}

// Test a QP with indefinite Hessian, but unique minimum.
// min x(0)² - x(1)²
// s.t x(1) = 1
GTEST_TEST(testEqualityConstrainedQPSolver, testIndefiniteHessian) {
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<2>("x");
  prog.AddCost(x(0) * x(0) - x(1) * x(1));
  auto constraint = prog.AddLinearConstraint(x(1) == 1);
  EqualityConstrainedQPSolver solver;
  auto result = solver.Solve(prog, {}, {});
  EXPECT_TRUE(result.is_success());
  EXPECT_TRUE(CompareMatrices(result.GetSolution(x), Eigen::Vector2d(0, 1),
                              1E-12, MatrixCompareType::absolute));
  EXPECT_NEAR(result.get_optimal_cost(), -1, 1E-12);
  EXPECT_TRUE(
      CompareMatrices(result.GetDualSolution(constraint), Vector1d(-2), 1E-12));
}

// Test a QP with positive semidefinite Hessian, but unbounded objective.
// min x(0)² - 2 * x(1)
// s.t x(0) = 1
GTEST_TEST(testEqualityConstrainedQPSolver, testPSDhessianUnbounded) {
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<2>("x");
  prog.AddCost(x(0) * x(0) - 2 * x(1));
  prog.AddLinearConstraint(x(0) == 1);
  EqualityConstrainedQPSolver solver;
  auto result = solver.Solve(prog, {}, {});
  EXPECT_EQ(result.get_solution_result(), SolutionResult::kUnbounded);
  EXPECT_EQ(result.get_optimal_cost(), MathematicalProgram::kUnboundedCost);
}

// Test a QP with negative definite Hessian, but with a unique optimum.
// min -x(0)² - x(1)²
// s.t x(0) + x(1) = 2
//     x(0) - x(1) = 3
GTEST_TEST(testEqualityConstrainedQPSolver, testNegativeHessianUniqueOptimum) {
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<2>("x");
  prog.AddCost(-x(0) * x(0) - x(1) * x(1));
  Eigen::Matrix2d Aeq;
  Aeq << 1, 1, 1, -1;
  auto constraint =
      prog.AddLinearEqualityConstraint(Aeq, Eigen::Vector2d(2, 3), x);
  EqualityConstrainedQPSolver solver;
  auto result = solver.Solve(prog, {}, {});
  EXPECT_TRUE(result.is_success());
  EXPECT_TRUE(CompareMatrices(result.GetSolution(x), Eigen::Vector2d(2.5, -0.5),
                              1E-12, MatrixCompareType::absolute));
  EXPECT_NEAR(result.get_optimal_cost(), -6.5, 1E-12);
  EXPECT_TRUE(CompareMatrices(result.GetDualSolution(constraint),
                              Eigen::Vector2d(-2, -3), 1e-12));
}

// Test a QP with negative definite Hessian, and an unbounded objective.
// min -x(0)² - x(1)²
// s.t x(0) + x(1) == 1
GTEST_TEST(testEqualityConstrainedQPSolver, testNegativeHessianUnbounded) {
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<2>("x");
  prog.AddCost(-x(0) * x(0) - x(1) * x(1));
  prog.AddLinearConstraint(x(0) + x(1) == 1);
  EqualityConstrainedQPSolver solver;
  auto result = solver.Solve(prog, {}, {});
  EXPECT_EQ(result.get_solution_result(), SolutionResult::kUnbounded);
  EXPECT_EQ(result.get_optimal_cost(), MathematicalProgram::kUnboundedCost);
}

// Test a QP with positive semidefinite Hessian (not strictly positive
// definite), and a unique minimum.
// min x(0)² + 2 * x(0) + 3 * x(1)
// s.t x(0) + 2 * x(1) == 1
GTEST_TEST(testEqualityConstrainedQPSolver, testPSDHessianUniqueOptimal) {
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<2>("x");
  prog.AddCost(x(0) * x(0) + 2 * x(0) + 3 * x(1));
  auto constraint = prog.AddLinearConstraint(x(0) + 2 * x(1) == 1);
  EqualityConstrainedQPSolver solver;
  auto result = solver.Solve(prog, {}, {});
  EXPECT_TRUE(result.is_success());
  EXPECT_TRUE(CompareMatrices(result.GetSolution(x),
                              Eigen::Vector2d(-1.0 / 4, 5.0 / 8), 1E-12,
                              MatrixCompareType::absolute));
  EXPECT_NEAR(result.get_optimal_cost(), 23.0 / 16.0, 1E-12);
  EXPECT_TRUE(CompareMatrices(result.GetDualSolution(constraint), Vector1d(1.5),
                              1e-12));
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
  EqualityConstrainedQPSolver solver;
  auto result = solver.Solve(prog, {}, {});
  EXPECT_EQ(result.get_solution_result(),
            SolutionResult::kInfeasibleConstraints);
  EXPECT_EQ(result.get_optimal_cost(),
            MathematicalProgram::kGlobalInfeasibleCost);
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
  EqualityConstrainedQPSolver solver;
  prog.SetSolverOption(EqualityConstrainedQPSolver::id(), "FeasibilityTol",
                       1E-7);
  auto result = solver.Solve(prog, {}, {});
  EXPECT_EQ(result.get_solution_result(),
            SolutionResult::kInfeasibleConstraints);
  EXPECT_EQ(result.get_optimal_cost(),
            MathematicalProgram::kGlobalInfeasibleCost);

  // Now increase the feasibility tolerance.
  double tol = 1E-6;
  prog.SetSolverOption(EqualityConstrainedQPSolver::id(),
                       EqualityConstrainedQPSolver::FeasibilityTolOptionName(),
                       tol);
  result = solver.Solve(prog, {}, {});
  EXPECT_TRUE(result.is_success());
  const Eigen::Vector2d x_val = result.GetSolution(x);
  const Eigen::Vector3d cnstr_val(x_val(0) + 2 * x_val(1), x_val(0) - x_val(1),
                                  x_val(0) + x_val(1));
  EXPECT_TRUE(CompareMatrices(cnstr_val, Eigen::Vector3d(1, -2, 1E-6), tol,
                              MatrixCompareType::absolute));
  EXPECT_NEAR(result.get_optimal_cost(), 3, 1E-6);

  // Now solve with a low feasibility tolerance again by passing the option in
  // the Solver function. The result should be infeasible.
  MathematicalProgramResult math_prog_result;
  SolverOptions solver_options;
  // The input solver option (1E-7) in `Solve` function takes priority over the
  // option stored in the prog (1E-6).
  solver_options.SetOption(
      EqualityConstrainedQPSolver::id(),
      EqualityConstrainedQPSolver::FeasibilityTolOptionName(), 0.1 * tol);
  solver.Solve(prog, {}, solver_options, &math_prog_result);
  EXPECT_FALSE(math_prog_result.is_success());
}

// min x'*x + x0 + x1 + 1
// The solution is x0 = x1 = -.5, with optimal value .5.
GTEST_TEST(testEqualityConstrainedQPSolver, testLinearCost) {
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<2>("x");

  prog.AddQuadraticCost(x.transpose() * x);
  prog.AddLinearCost(x(0) + x(1) + 1);

  MathematicalProgramResult result;
  result = Solve(prog);
  EXPECT_TRUE(result.is_success());

  EXPECT_TRUE(
      CompareMatrices(result.GetSolution(x), Eigen::Vector2d(-.5, -.5), 1e-6));
  EXPECT_EQ(result.get_optimal_cost(), .5);
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

GTEST_TEST(EqualityConstrainedQPSolverDualSolutionTest, DualSolution1) {
  EqualityConstrainedQPSolver solver;
  TestEqualityConstrainedQPDualSolution1(solver);
}

GTEST_TEST(EqualityConstrainedQPSolverDualSolutionTest, DualSolution2) {
  EqualityConstrainedQPSolver solver;
  TestEqualityConstrainedQPDualSolution2(solver);
}

}  // namespace test
}  // namespace solvers
}  // namespace drake
