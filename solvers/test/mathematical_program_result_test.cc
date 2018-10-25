#include "drake/solvers/mathematical_program_result.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"

namespace drake {
namespace solvers {
namespace {
GTEST_TEST(MathematicalProgramResultTest, DefaultConstructor) {
  MathematicalProgramResult result;
  EXPECT_EQ(result.get_solution_result(), SolutionResult::kUnknownError);
  EXPECT_EQ(result.get_x_val().size(), 0);
  EXPECT_TRUE(std::isnan(result.get_optimal_cost()));
  EXPECT_NO_THROW(
      result.get_solver_details().GetValueOrThrow<NoSolverDetails>());
}

GTEST_TEST(MathematicalProgramResultTest, Setters) {
  MathematicalProgramResult result;
  result.set_solution_result(SolutionResult::kSolutionFound);
  const Eigen::Vector2d x_val(0, 0);
  result.set_x_val(x_val);
  const double cost = 1;
  result.set_optimal_cost(cost);
  result.set_solver_id(SolverId("foo"));
  EXPECT_EQ(result.get_solution_result(), SolutionResult::kSolutionFound);
  EXPECT_TRUE(CompareMatrices(result.get_x_val(), x_val));
  EXPECT_EQ(result.get_optimal_cost(), cost);
  EXPECT_EQ(result.get_solver_id().name(), "foo");
  EXPECT_NO_THROW(
      result.get_solver_details().GetValueOrThrow<NoSolverDetails>());
}

struct DummySolverDetails {
  DummySolverDetails() : data(0) {}
  int data;
};

GTEST_TEST(MathematicalProgramResultTest, SetSolverDetails) {
  MathematicalProgramResult result;
  const int data = 1;
  DummySolverDetails& dummy_solver_details =
      result.SetSolverDetailsType<DummySolverDetails>();
  dummy_solver_details.data = data;
  EXPECT_EQ(result.get_solver_details().GetValue<DummySolverDetails>().data,
            data);
  // Now we test if we call SetSolverDetailsType again, it doesn't allocate new
  // memory.
  const systems::AbstractValue* details = &(result.get_solver_details());
  dummy_solver_details = result.SetSolverDetailsType<DummySolverDetails>();
  EXPECT_EQ(details, &(result.get_solver_details()));
}
}  // namespace
}  // namespace solvers
}  // namespace drake
