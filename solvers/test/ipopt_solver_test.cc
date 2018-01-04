#include "drake/solvers/ipopt_solver.h"

#include <gtest/gtest.h>

#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/test/linear_program_examples.h"
#include "drake/solvers/test/mathematical_program_test_util.h"
#include "drake/solvers/test/quadratic_program_examples.h"

namespace drake {
namespace solvers {
namespace test {

TEST_P(LinearProgramTest, TestLP) {
  IpoptSolver solver;
  prob()->RunProblem(&solver);
}

INSTANTIATE_TEST_CASE_P(
    IpoptTest, LinearProgramTest,
    ::testing::Combine(::testing::ValuesIn(linear_cost_form()),
                       ::testing::ValuesIn(linear_constraint_form()),
                       ::testing::ValuesIn(linear_problems())));

TEST_F(InfeasibleLinearProgramTest0, TestIpopt) {
  prog_->SetInitialGuessForAllVariables(Eigen::Vector2d(1, 2));
  IpoptSolver solver;
  if (solver.available()) {
    const auto solver_result = solver.Solve(*prog_);
    EXPECT_EQ(solver_result, SolutionResult::kInfeasibleConstraints);
    const Eigen::Vector2d x_val =
        prog_->GetSolution(prog_->decision_variables());
    EXPECT_NEAR(prog_->GetOptimalCost(), -x_val(0) - x_val(1), 1E-7);
  }
}

TEST_F(UnboundedLinearProgramTest0, TestIpopt) {
  prog_->SetInitialGuessForAllVariables(Eigen::Vector2d(1, 2));
  prog_->SetSolverOption(IpoptSolver::id(), "diverging_iterates_tol", 1E3);
  prog_->SetSolverOption(IpoptSolver::id(), "max_iter", 1000);
  IpoptSolver solver;
  if (solver.available()) {
    const auto solver_result = solver.Solve(*prog_);
    EXPECT_EQ(solver_result, SolutionResult::kUnbounded);
    EXPECT_EQ(prog_->GetOptimalCost(),
              -std::numeric_limits<double>::infinity());
  }
}

TEST_P(QuadraticProgramTest, TestQP) {
  IpoptSolver solver;
  prob()->RunProblem(&solver);
}

INSTANTIATE_TEST_CASE_P(
    IpoptTest, QuadraticProgramTest,
    ::testing::Combine(::testing::ValuesIn(quadratic_cost_form()),
                       ::testing::ValuesIn(linear_constraint_form()),
                       ::testing::ValuesIn(quadratic_problems())));

GTEST_TEST(QPtest, TestUnitBallExample) {
  IpoptSolver solver;
  if (solver.available()) {
    TestQPonUnitBallExample(solver);
  }
}
}  // namespace test
}  // namespace solvers
}  // namespace drake
