#include "drake/solvers/mathematical_program_result.h"

#include <limits>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/common/test_utilities/symbolic_test_util.h"
#include "drake/solvers/cost.h"
#include "drake/solvers/osqp_solver.h"
#include "drake/solvers/snopt_solver.h"

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
  DRAKE_EXPECT_THROWS_MESSAGE(result.get_abstract_solver_details(),
                              "The solver_details has not been set yet.");
}

TEST_F(MathematicalProgramResultTest, Setters) {
  MathematicalProgramResult result;
  result.set_decision_variable_index(decision_variable_index_);
  EXPECT_EQ(result.get_decision_variable_index(), decision_variable_index_);
  EXPECT_TRUE(CompareMatrices(
      result.get_x_val(),
      Eigen::VectorXd::Constant(decision_variable_index_.size(),
                                std::numeric_limits<double>::quiet_NaN())));
  result.set_solution_result(SolutionResult::kSolutionFound);
  EXPECT_TRUE(result.is_success());
  const Eigen::Vector2d x_val(0, 1);
  result.set_x_val(x_val);
  result.AddSuboptimalSolution(0.1, Eigen::Vector2d(1, 2));
  EXPECT_TRUE(CompareMatrices(result.get_x_val(), x_val));
  EXPECT_EQ(result.num_suboptimal_solution(), 1);
  EXPECT_EQ(result.GetSuboptimalSolution(x0_, 0), 1);
  EXPECT_EQ(result.GetSuboptimalSolution(x1_, 0), 2);
  EXPECT_EQ(result.get_suboptimal_objective(0), 0.1);
  DRAKE_EXPECT_THROWS_MESSAGE(result.set_x_val(Eigen::Vector3d::Zero()),
                              "MathematicalProgramResult::set_x_val, the "
                              "dimension of x_val is 3, expected 2");
  const double cost = 1;
  result.set_optimal_cost(cost);
  result.set_solver_id(SolverId("foo"));
  EXPECT_EQ(result.get_optimal_cost(), cost);
  EXPECT_EQ(result.get_solver_id().name(), "foo");
  EXPECT_TRUE(CompareMatrices(result.GetSolution(), x_val));

  result.SetSolution(x0_, 0.123);
  EXPECT_EQ(result.GetSolution(x0_), 0.123);
  symbolic::Variable unregistered("unregistered");
  EXPECT_THROW(result.SetSolution(unregistered, 0.456), std::exception);
}

TEST_F(MathematicalProgramResultTest, GetSolution) {
  // Test GetSolution function.
  MathematicalProgramResult result;
  result.set_decision_variable_index(decision_variable_index_);
  result.set_solution_result(SolutionResult::kSolutionFound);
  const Eigen::Vector2d x_val(0, 1);
  result.set_x_val(x_val);

  EXPECT_EQ(result.GetSolution(x0_), x_val(0));
  EXPECT_EQ(result.GetSolution(x1_), x_val(1));
  EXPECT_EQ(result.GetSolution(Vector2<symbolic::Variable>(x0_, x1_)), x_val);
  EXPECT_TRUE(CompareMatrices(result.get_x_val(), x_val));
  EXPECT_TRUE(CompareMatrices(result.GetSolution(), x_val));

  // Getting solution for a variable y not in decision_variable_index_.
  symbolic::Variable y("y");
  DRAKE_EXPECT_THROWS_MESSAGE(
      result.GetSolution(y),
      "GetVariableValue: y is not captured by the variable_index map.");

  // Get a solution of an Expression (with additional Variables).
  const symbolic::Variable x_extra{"extra"};
  const symbolic::Expression e{x0_ + x_extra};
  EXPECT_TRUE(
      result.GetSolution(e).EqualTo(symbolic::Expression{x_val(0) + x_extra}));
  const Vector2<symbolic::Expression> m{x0_ + x_extra, x1_ * x_extra};
  const Vector2<symbolic::Expression> msol = result.GetSolution(m);
  EXPECT_TRUE(msol[0].EqualTo(x_val(0) + x_extra));
  EXPECT_TRUE(msol[1].EqualTo(x_val(1) * x_extra));
}

TEST_F(MathematicalProgramResultTest, GetSolutionPolynomial) {
  // Test GetSolution on symbolic::Polynomial.
  MathematicalProgramResult result;
  result.set_decision_variable_index(decision_variable_index_);
  const Eigen::Vector2d x_val(2, 1);
  result.set_x_val(x_val);

  // t1 and t2 are indeterminates.
  symbolic::Variable t1{"t1"};
  symbolic::Variable t2{"t2"};

  // p1 doesn't contain any decision variable. Its coefficients are constant.
  const symbolic::Polynomial p1(2 * t1 * t1 + t2, {t1, t2});
  EXPECT_PRED2(symbolic::test::PolyEqual, p1, result.GetSolution(p1));

  // p2's coefficients are expressions of x0 and x1
  const symbolic::Polynomial p2(
      (1 + x0_ * x1_) * t1 * t1 + 2 * sin(x1_) * t1 * t2 + 3 * x0_, {t1, t2});
  EXPECT_PRED2(
      symbolic::test::PolyEqual,
      symbolic::Polynomial(
          {{symbolic::Monomial(t1, 2), 1 + x_val(0) * x_val(1)},
           {symbolic::Monomial({{t1, 1}, {t2, 1}}), 2 * std::sin(x_val(1))},
           {symbolic::Monomial(), 3 * x_val(0)}}),
      result.GetSolution(p2));

  // p3's indeterminates contain x0, expect to throw an error.
  DRAKE_EXPECT_THROWS_MESSAGE(
      result.GetSolution(symbolic::Polynomial(x0_ * t1 + 1, {x0_, t1})),
      ".*x0 is an indeterminate in the polynomial.*");
}

TEST_F(MathematicalProgramResultTest, DualSolution) {
  MathematicalProgramResult result;
  auto bb_con = std::make_shared<BoundingBoxConstraint>(Eigen::Vector2d(0, 1),
                                                        Eigen::Vector2d(2, 4));
  Binding<BoundingBoxConstraint> binding1(
      bb_con, Vector2<symbolic::Variable>(x0_, x1_));
  const Eigen::Vector2d dual_solution1(5, 6);
  result.set_dual_solution(binding1, dual_solution1);
  EXPECT_TRUE(
      CompareMatrices(result.GetDualSolution(binding1), dual_solution1));

  auto lin_con = std::make_shared<LinearConstraint>(Eigen::Matrix2d::Identity(),
                                                    Eigen::Vector2d(-1, -3),
                                                    Eigen::Vector2d(2, 4));
  Binding<LinearConstraint> binding2(lin_con,
                                     Vector2<symbolic::Variable>(x0_, x1_));
  const Eigen::Vector2d dual_solution2(3, -2);
  result.set_dual_solution(binding2, dual_solution2);
  EXPECT_TRUE(
      CompareMatrices(result.GetDualSolution(binding2), dual_solution2));

  auto lin_eq_con = std::make_shared<LinearEqualityConstraint>(
      Eigen::Matrix2d::Identity(), Eigen::Vector2d(2, 4));
  Binding<LinearEqualityConstraint> binding3(
      lin_eq_con, Vector2<symbolic::Variable>(x0_, x1_));
  const Eigen::Vector2d dual_solution3(4, -1);
  result.set_dual_solution(binding3, dual_solution3);
  EXPECT_TRUE(
      CompareMatrices(result.GetDualSolution(binding3), dual_solution3));

  // GetDualSolution for a binding whose dual solution has not been set yet.
  Binding<LinearEqualityConstraint> binding4(
      lin_eq_con, Vector2<symbolic::Variable>(x1_, x0_));
  DRAKE_EXPECT_THROWS_MESSAGE(
      result.GetDualSolution(binding4),
      fmt::format("Either this constraint does not belong to the "
                  "mathematical program for which the result is obtained, or "
                  "{} does not currently support getting dual solution yet.",
                  result.get_solver_id()));
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
    EXPECT_EQ(result.get_solution_result(),
              SolutionResult::kInfeasibleConstraints);
    EXPECT_EQ(result.GetSolution(x).size(), 2);
    EXPECT_EQ(result.GetSolution().size(), 2);

    EXPECT_EQ(result.get_optimal_cost(),
              MathematicalProgram::kGlobalInfeasibleCost);
  }
}

GTEST_TEST(TestMathematicalProgramResult, GetInfeasibleConstraintNames) {
  if (SnoptSolver::is_available() && SnoptSolver::is_enabled()) {
    MathematicalProgram prog;
    auto x = prog.NewContinuousVariables<1>();
    auto b0 = prog.AddBoundingBoxConstraint(0, 0, x);
    auto b1 = prog.AddBoundingBoxConstraint(1, 1, x);

    SnoptSolver solver;
    MathematicalProgramResult result = solver.Solve(prog, {}, {});
    EXPECT_FALSE(result.is_success());

    std::vector<std::string> infeasible =
        result.GetInfeasibleConstraintNames(prog);
    EXPECT_EQ(infeasible.size(), 1);

    // If no description is set, we should see the NiceTypeName of the
    // Constraint.
    auto matcher = [](const std::string& s, const std::string& re) {
      return std::regex_match(s, std::regex(re));
    };
    EXPECT_PRED2(matcher, infeasible[0],
                 "drake::solvers::BoundingBoxConstraint.*");

    // If a description for the constraint has been set, then that description
    // should be returned instead. There is no reason a priori for b0 or b1 to
    // be the infeasible one, so set both descriptions.
    b0.evaluator()->set_description("Test");
    b1.evaluator()->set_description("Test");
    infeasible = result.GetInfeasibleConstraintNames(prog);
    EXPECT_PRED2(matcher, infeasible[0], "Test.*");
  }
}

GTEST_TEST(TestMathematicalProgramResult, GetInfeasibleConstraintBindings) {
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<3>();
  // Choose constraint values such that x=0 should violate all constraints.
  const Eigen::Vector2d value1 = Eigen::Vector2d::Constant(10);
  const Eigen::Vector3d value2 = Eigen::Vector3d::Constant(100);
  auto constraint1 = prog.AddBoundingBoxConstraint(value1, value1, x.head<2>());
  auto constraint2 = prog.AddBoundingBoxConstraint(value2, value2, x);
  SnoptSolver solver;
  if (solver.is_available() && solver.is_enabled()) {
    const auto result = solver.Solve(prog);
    EXPECT_FALSE(result.is_success());
    const double tol = 1e-4;
    using Bindings = std::vector<Binding<Constraint>>;
    const Bindings infeasible_bindings =
        result.GetInfeasibleConstraints(prog, tol);
    const Eigen::Vector3d x_sol = result.GetSolution(x);
    const bool violates_constraint1 =
        !constraint1.evaluator()->CheckSatisfied(x_sol.head<2>(), tol);
    const bool violates_constraint2 =
        !constraint2.evaluator()->CheckSatisfied(x_sol, tol);
    // At least one of the constraints should be violated.
    EXPECT_TRUE(violates_constraint1 || violates_constraint2);
    // Ensure we only have one occurrence of each in the order in which we
    // added them.
    Bindings bindings_expected;
    if (violates_constraint1) {
      bindings_expected.push_back(constraint1);
    }
    if (violates_constraint2) {
      bindings_expected.push_back(constraint2);
    }
    EXPECT_EQ(infeasible_bindings, bindings_expected);
    // If I relax the tolerance, then GetInfeasibleConstraintBindings returns an
    // empty vector.
    const std::vector<Binding<Constraint>> infeasible_bindings_relaxed =
        result.GetInfeasibleConstraints(prog, 1000);
    EXPECT_EQ(infeasible_bindings_relaxed.size(), 0);
  }
}

}  // namespace
}  // namespace solvers
}  // namespace drake
