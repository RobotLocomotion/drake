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
  EXPECT_NO_THROW(result.solver_details().GetValueOrThrow<NoSolverDetails>());
}

GTEST_TEST(MathematicalProgramResultTest, Constructor) {
  MathematicalProgramResult result;
  result.get_mutable_result() = SolutionResult::kSolutionFound;
  const Eigen::Vector2d x_val(0, 0);
  result.get_mutable_x_val() = x_val;
  const double cost = 1;
  result.get_mutable_optimal_cost() = cost;
  result.get_mutable_solver_id() = SolverId("foo");
  EXPECT_EQ(result.result(), SolutionResult::kSolutionFound);
  EXPECT_TRUE(CompareMatrices(result.x_val(), x_val));
  EXPECT_EQ(result.optimal_cost(), cost);
  EXPECT_EQ(result.solver_id().name(), "foo");
  EXPECT_NO_THROW(result.solver_details().GetValueOrThrow<NoSolverDetails>());
}

struct DummySolverDetails {
  explicit DummySolverDetails(int m_data) : data(m_data) {}
  int data;
};

GTEST_TEST(MathematicalProgramResultTest, SetSolverDetails) {
  MathematicalProgramResult result;
  const int data = 1;
  auto dummy_solver_result = systems::AbstractValue::Make<DummySolverDetails>(
      DummySolverDetails(data));
  result.SetSolverDetails(std::move(dummy_solver_result));
  EXPECT_EQ(result.solver_details().GetValue<DummySolverDetails>().data, data);
  EXPECT_FALSE(dummy_solver_result);
}
}  // namespace
}  // namespace solvers
}  // namespace drake
