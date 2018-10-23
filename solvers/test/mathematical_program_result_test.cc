#include "drake/solvers/mathematical_program_result.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"

namespace drake {
namespace solvers {
namespace {
GTEST_TEST(MathematicalProgramResultTest, DefaultConstructor) {
  MathematicalProgramResult result;
  EXPECT_EQ(result.result(), SolutionResult::kUnknownError);
  EXPECT_EQ(result.x_val().size(), 0);
  EXPECT_TRUE(std::isnan(result.optimal_cost()));
  EXPECT_EQ(result.solver_id().name(), "unknown");
  DRAKE_EXPECT_THROWS_MESSAGE(result.solver_details(), std::logic_error,
                              "The solver_details has not been set yet.");
}

GTEST_TEST(MathematicalProgramResultTest, Constructor) {
  MathematicalProgramResult result(SolutionResult::kSolutionFound,
                                   Eigen::Vector2d::Zero(), 1, SolverId("foo"));
  EXPECT_EQ(result.result(), SolutionResult::kSolutionFound);
  EXPECT_TRUE(CompareMatrices(result.x_val(), Eigen::Vector2d::Zero()));
  EXPECT_EQ(result.optimal_cost(), 1);
  EXPECT_EQ(result.solver_id().name(), "foo");
  EXPECT_NO_THROW(result.solver_details().GetValueOrThrow<NoSolverDetails>());
}

struct DummySolverDetails {
  DummySolverDetails(int m_data) : data(m_data) {}
  int data;
};

GTEST_TEST(MathematicalProgramResultTest, SetSolverDetails) {
  MathematicalProgramResult result;
  auto dummy_solver_result =
      systems::AbstractValue::Make<DummySolverDetails>(1);
  result.SetSolverDetails(std::move(dummy_solver_result));
  EXPECT_EQ(result.solver_details().GetValue<DummySolverDetails>().data, 1);
  EXPECT_FALSE(dummy_solver_result);
}
}  // namespace
}  // namespace solvers
}  // namespace drake
