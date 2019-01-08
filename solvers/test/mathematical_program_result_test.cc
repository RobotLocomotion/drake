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
  DRAKE_EXPECT_THROWS_MESSAGE(result.get_solver_details(), std::logic_error,
                              "The solver_details has not been set yet.");
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
}

struct DummySolverDetails {
  int data{0};
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
  // First we check the address of (result.get_solver_details()) is unchanged.
  const AbstractValue* details = &(result.get_solver_details());
  dummy_solver_details = result.SetSolverDetailsType<DummySolverDetails>();
  EXPECT_EQ(details, &(result.get_solver_details()));
  // Now we check that the value in the solver details are unchanged, note that
  // the default value for data is 0, as in the constructor of
  // DummySolverDetails, so if the constructor were called,
  // dummy_solver_details.data won't be equal to 1.
  dummy_solver_details = result.SetSolverDetailsType<DummySolverDetails>();
  EXPECT_EQ(result.get_solver_details().GetValue<DummySolverDetails>().data,
            data);
}

GTEST_TEST(MathematicalProgramResultTest, ConvertToSolverResult) {
  MathematicalProgramResult result;
  result.set_solver_id(SolverId("foo"));
  result.set_optimal_cost(2);
  // The x_val is not set. So solver_result.decision_variable_values should be
  // empty.
  SolverResult solver_result = result.ConvertToSolverResult();
  EXPECT_FALSE(solver_result.decision_variable_values());
  // Now set x_val.
  result.set_x_val(Eigen::Vector2d::Ones());
  solver_result = result.ConvertToSolverResult();
  EXPECT_EQ(result.get_solver_id(), solver_result.solver_id());
  EXPECT_TRUE(CompareMatrices(
      result.get_x_val(), solver_result.decision_variable_values().value()));
  EXPECT_EQ(result.get_optimal_cost(), solver_result.optimal_cost());
}
}  // namespace
}  // namespace solvers
}  // namespace drake
