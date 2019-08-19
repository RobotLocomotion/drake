#include "drake/solvers/mathematical_program_result.h"

#include <limits>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/solvers/cost.h"
#include "drake/solvers/osqp_solver.h"

namespace drake {
namespace solvers {
namespace {
class MathematicalProgramResultTest : public ::testing::Test {
 public:
  MathematicalProgramResultTest()
      : x0_{"x0"}, x1_{"x1"}, decision_variable_index_{} {
    decision_variable_index_.emplace(x0_.get_id(), 0);
    decision_variable_index_.emplace(x1_.get_id(), 1);
  }

 protected:
  symbolic::Variable x0_;
  symbolic::Variable x1_;
  std::unordered_map<symbolic::Variable::Id, int> decision_variable_index_;
};

TEST_F(MathematicalProgramResultTest, DefaultConstructor) {
  MathematicalProgramResult result;
  EXPECT_FALSE(result.is_success());
  EXPECT_EQ(result.get_x_val().size(), 0);
  EXPECT_TRUE(std::isnan(result.get_optimal_cost()));
  EXPECT_EQ(result.num_suboptimal_solution(), 0);
  DRAKE_EXPECT_THROWS_MESSAGE(
      result.get_abstract_solver_details(), std::logic_error,
      "The solver_details has not been set yet.");
}

TEST_F(MathematicalProgramResultTest, Setters) {
  MathematicalProgramResult result;
  result.set_decision_variable_index(decision_variable_index_);
  EXPECT_TRUE(CompareMatrices(
      result.get_x_val(),
      Eigen::VectorXd::Constant(decision_variable_index_.size(),
                                std::numeric_limits<double>::quiet_NaN())));
  result.set_solution_result(SolutionResult::kSolutionFound);
  const Eigen::Vector2d x_val(0, 1);
  result.set_x_val(x_val);
  result.AddSuboptimalSolution(0.1, Eigen::Vector2d(1, 2));
  EXPECT_TRUE(CompareMatrices(result.get_x_val(), x_val));
  EXPECT_TRUE(CompareMatrices(result.GetSolution(), x_val));
  EXPECT_EQ(result.GetSolution(x0_), x_val(0));
  EXPECT_EQ(result.GetSolution(x1_), x_val(1));
  EXPECT_EQ(result.GetSolution(Vector2<symbolic::Variable>(x0_, x1_)), x_val);
  EXPECT_EQ(result.num_suboptimal_solution(), 1);
  EXPECT_EQ(result.GetSuboptimalSolution(x0_, 0), 1);
  EXPECT_EQ(result.GetSuboptimalSolution(x1_, 0), 2);
  EXPECT_EQ(result.get_suboptimal_objective(0), 0.1);
  DRAKE_EXPECT_THROWS_MESSAGE(result.set_x_val(Eigen::Vector3d::Zero()),
                              std::invalid_argument,
                              "MathematicalProgramResult::set_x_val, the "
                              "dimension of x_val is 3, expected 2");
  const double cost = 1;
  result.set_optimal_cost(cost);
  result.set_solver_id(SolverId("foo"));
  EXPECT_TRUE(result.is_success());
  EXPECT_TRUE(CompareMatrices(result.get_x_val(), x_val));
  EXPECT_TRUE(CompareMatrices(result.GetSolution(), x_val));
  EXPECT_EQ(result.get_optimal_cost(), cost);
  EXPECT_EQ(result.get_solver_id().name(), "foo");

  // Getting solution for a variable y not in decision_variable_index_.
  symbolic::Variable y("y");
  DRAKE_EXPECT_THROWS_MESSAGE(
      result.GetSolution(y), std::invalid_argument,
      "GetVariableValue: y is not captured by the variable_index map.");

  // Get a solution of an Expression (with additional Variables).
  const symbolic::Variable x_extra{"extra"};
  const symbolic::Expression e{x0_ + x_extra};
  EXPECT_TRUE(result.GetSolution(e).EqualTo(symbolic::Expression{x_val(0) +
  x_extra}));
  const Vector2<symbolic::Expression> m{x0_ + x_extra, x1_*x_extra};
  const Vector2<symbolic::Expression> msol = result.GetSolution(m);
  EXPECT_TRUE(msol[0].EqualTo(x_val(0) + x_extra));
  EXPECT_TRUE(msol[1].EqualTo(x_val(1) * x_extra));
}

struct DummySolverDetails {
  int data{0};
};
struct DummySolver {
  using Details = DummySolverDetails;
};

TEST_F(MathematicalProgramResultTest, SetSolverDetails) {
  MathematicalProgramResult result;
  result.set_decision_variable_index(decision_variable_index_);
  const int data = 1;
  DummySolverDetails& dummy_solver_details =
      result.SetSolverDetailsType<DummySolverDetails>();
  dummy_solver_details.data = data;
  EXPECT_EQ(result.get_solver_details<DummySolver>().data, data);
  // Now we test if we call SetSolverDetailsType again, it doesn't allocate new
  // memory.  First we check that the address is unchanged.
  const AbstractValue* details = &(result.get_abstract_solver_details());
  dummy_solver_details = result.SetSolverDetailsType<DummySolverDetails>();
  EXPECT_EQ(details, &(result.get_abstract_solver_details()));
  // Now we check that the value in the details is unchanged, note that the
  // default value for data is 0, as in the constructor of Details, so if the
  // constructor were called, dummy_solver_details.data won't be equal to 1.
  dummy_solver_details = result.SetSolverDetailsType<DummySolverDetails>();
  EXPECT_EQ(result.get_solver_details<DummySolver>().data, data);
}

TEST_F(MathematicalProgramResultTest, EvalBinding) {
  MathematicalProgramResult result;
  result.set_decision_variable_index(decision_variable_index_);
  const Eigen::Vector2d x_val(0, 1);
  result.set_x_val(x_val);
  const Binding<LinearCost> cost{std::make_shared<LinearCost>(Vector1d(2), 0),
                                 Vector1<symbolic::Variable>(x1_)};
  EXPECT_TRUE(CompareMatrices(result.EvalBinding(cost), Vector1d(2)));
}

GTEST_TEST(TestMathematicalProgramResult, InfeasibleProblem) {
  // Test if we can query the information in the result when the problem is
  // infeasible.
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<2>();
  prog.AddLinearConstraint(x(0) + x(1) <= 1);
  prog.AddLinearConstraint(x(0) >= 1);
  prog.AddLinearConstraint(x(1) >= 1);
  prog.AddQuadraticCost(x.dot(x.cast<symbolic::Expression>()));

  MathematicalProgramResult result;
  OsqpSolver osqp_solver;
  if (osqp_solver.available()) {
    const Eigen::VectorXd x_guess = Eigen::Vector2d::Zero();
    osqp_solver.Solve(prog, x_guess, {}, &result);
    EXPECT_TRUE(CompareMatrices(
        result.GetSolution(x),
        Eigen::Vector2d::Constant(std::numeric_limits<double>::quiet_NaN())));
    EXPECT_TRUE(std::isnan(result.GetSolution(x(0))));
    EXPECT_TRUE(std::isnan(result.GetSolution(x(1))));
    EXPECT_EQ(result.get_optimal_cost(),
              MathematicalProgram::kGlobalInfeasibleCost);
  }
}

}  // namespace
}  // namespace solvers
}  // namespace drake
