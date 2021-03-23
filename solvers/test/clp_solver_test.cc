#include "drake/solvers/clp_solver.h"

#include <gtest/gtest.h>

#include "drake/solvers/test/linear_program_examples.h"

namespace drake {
namespace solvers {
namespace test {
TEST_P(LinearProgramTest, TestLP) {
  ClpSolver solver;
  prob()->RunProblem(&solver);
}

INSTANTIATE_TEST_SUITE_P(
    ClpTest, LinearProgramTest,
    ::testing::Combine(::testing::ValuesIn(linear_cost_form()),
                       ::testing::ValuesIn(linear_constraint_form()),
                       ::testing::ValuesIn(linear_problems())));

TEST_F(InfeasibleLinearProgramTest0, TestInfeasible) {
  ClpSolver solver;
  if (solver.available()) {
    auto result = solver.Solve(*prog_, {}, {});
    EXPECT_EQ(result.get_solution_result(),
              SolutionResult::kInfeasibleConstraints);
    EXPECT_TRUE(std::isinf(result.get_optimal_cost()));
    EXPECT_GT(result.get_optimal_cost(), 0.);
    // This code is defined in ClpModel::status()
    const int CLP_INFEASIBLE = 1;
    EXPECT_EQ(result.get_solver_details<ClpSolver>().status, CLP_INFEASIBLE);
  }
}

TEST_F(UnboundedLinearProgramTest0, TestUnbounded) {
  ClpSolver solver;
  if (solver.available()) {
    auto result = solver.Solve(*prog_, {}, {});
    EXPECT_FALSE(result.is_success());
    EXPECT_EQ(result.get_solution_result(), SolutionResult::kUnbounded);
    EXPECT_TRUE(std::isinf(result.get_optimal_cost()));
    EXPECT_LT(result.get_optimal_cost(), 0.);
    // This code is defined in ClpModel::status()
    const int CLP_UNBOUNDED = 2;
    EXPECT_EQ(result.get_solver_details<ClpSolver>().status, CLP_UNBOUNDED);
  }
}
}  // namespace test
}  // namespace solvers
}  // namespace drake
