#include "drake/solvers/gurobi_solver.h"

#include <gtest/gtest.h>

#include "drake/common/eigen_matrix_compare.h"
#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/test/linear_program_examples.h"
#include "drake/solvers/test/mathematical_program_test_util.h"
#include "drake/solvers/test/quadratic_program_examples.h"

namespace drake {
namespace solvers {
namespace test {

TEST_P(LinearProgramTest, TestLP) {
  GurobiSolver solver;
  prob()->RunProblem(&solver);
}

INSTANTIATE_TEST_CASE_P(
    GurobiTest, LinearProgramTest,
    ::testing::Combine(::testing::ValuesIn(linear_cost_form()),
                       ::testing::ValuesIn(linear_constraint_form()),
                       ::testing::ValuesIn(linear_problems())));

TEST_F(InfeasibleLinearProgramTest0, TestGurobiInfeasible) {
  GurobiSolver solver;
  if (solver.available()) {
    // With dual reductions, gurobi may not be able to differentiate between
    // infeasible and unbounded.
    prog_->SetSolverOption(SolverType::kGurobi, "DualReductions", 1);
    SolutionResult result = solver.Solve(*prog_);
    EXPECT_EQ(result, SolutionResult::kInfeasible_Or_Unbounded);
    prog_->SetSolverOption(SolverType::kGurobi, "DualReductions", 0);
    result = solver.Solve(*prog_);
    EXPECT_EQ(result, SolutionResult::kInfeasibleConstraints);
  }
}

TEST_F(UnboundedLinearProgramTest0, TestGurobiUnbounded) {
  GurobiSolver solver;
  if (solver.available()) {
    // With dual reductions, gurobi may not be able to differentiate between
    // infeasible and unbounded.
    prog_->SetSolverOption(SolverType::kGurobi, "DualReductions", 1);
    SolutionResult result = solver.Solve(*prog_);
    EXPECT_EQ(result, SolutionResult::kInfeasible_Or_Unbounded);
    prog_->SetSolverOption(SolverType::kGurobi, "DualReductions", 0);
    result = solver.Solve(*prog_);
    EXPECT_EQ(result, SolutionResult::kUnbounded);
  }
}

TEST_P(QuadraticProgramTest, TestQP) {
  GurobiSolver solver;
  prob()->RunProblem(&solver);
}

INSTANTIATE_TEST_CASE_P(
    GurobiTest, QuadraticProgramTest,
    ::testing::Combine(::testing::ValuesIn(quadratic_cost_form()),
                       ::testing::ValuesIn(linear_constraint_form()),
                       ::testing::ValuesIn(quadratic_problems())));

GTEST_TEST(QPtest, TestUnitBallExample) {
  GurobiSolver solver;
  if (solver.available()) {
    TestQPonUnitBallExample(solver);
  }
}

GTEST_TEST(GurobiTest, TestInitialGuess) {
  GurobiSolver solver;
  if (solver.available()) {
    // Formulate a simple problem, supply an exactly-correct
    // initial guess, and solve.
    MathematicalProgram prog;
    auto x = prog.NewBinaryVariables<1>("x");
    prog.AddLinearCost(-1.0 * Eigen::VectorXd::Ones(1), x);
    Eigen::VectorXd x_expected = Eigen::VectorXd::Ones(1);
    prog.SetInitialGuess(x, x_expected);
    SolutionResult result = solver.Solve(prog);
    EXPECT_EQ(result, SolutionResult::kSolutionFound);
    const auto& x_value = prog.GetSolution(x);
    EXPECT_TRUE(CompareMatrices(x_value, x_expected, 1E-6,
                                MatrixCompareType::absolute));
  }
}
}  // namespace test
}  // namespace solvers
}  // namespace drake
