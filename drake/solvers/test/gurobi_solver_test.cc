#include "drake/solvers/gurobi_solver.h"

#include <gtest/gtest.h>

#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/test/mathematical_program_test_util.h"
#include "drake/solvers/test/optimization_examples.h"

namespace drake {
namespace solvers {
namespace test {
class GurobiLinearProgramTest : public LinearProgramTest {
};

TEST_P(GurobiLinearProgramTest, TestLP) {
  GurobiSolver solver;
  if (solver.available()) {
    RunSolver(prob()->prog(), solver);
    prob()->CheckSolution();
  }
}

INSTANTIATE_TEST_CASE_P(
    GurobiTest, GurobiLinearProgramTest,
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
}  // namespace test
}  // namespace solvers
}  // namespace drake
