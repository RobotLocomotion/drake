#include "drake/systems/optimization/system_constraint_adapter.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/math/autodiff_gradient.h"
#include "drake/solvers/solve.h"
#include "drake/systems/optimization/test/system_optimization_test_util.h"

namespace drake {
namespace systems {
const double kInf = std::numeric_limits<double>::infinity();
const double kEps = std::numeric_limits<double>::epsilon();

// Assumes that vars = [p; x(0); x(1); x(2)].
struct DummySystemSelector1 {
  template <typename T>
  void operator()(const System<T>&, const Eigen::Ref<const VectorX<T>>& vars,
                  Context<T>* context) {
    context->get_mutable_continuous_state_vector().SetFromVector(
        vars.template tail<3>());
    context->get_mutable_numeric_parameter(0).SetAtIndex(0, vars(0));
  }
};

GTEST_TEST(SystemConstraintAdapter, CreateSystemConstraintWrapper) {
  DummySystem<double> system;

  SystemConstraintAdapter adapter(&system);

  auto context = system.CreateDefaultContext();

  auto constraint1 = adapter.Create(system.constraint_index(), *context,
                                    DummySystemSelector1{}, 4);
  EXPECT_TRUE(
      CompareMatrices(constraint1->lower_bound(), Eigen::Vector2d(2, 0)));
  EXPECT_TRUE(
      CompareMatrices(constraint1->upper_bound(), Eigen::Vector2d(kInf, kInf)));

  EXPECT_EQ(constraint1->get_description(), "dummy_system_constraint");

  const Eigen::Vector4d var(2, 3, 4, 5);
  Eigen::VectorXd y;
  constraint1->Eval(var, &y);

  DummySystemSelector1 selector;
  selector.operator()<double>(system, var, context.get());
  Eigen::VectorXd y_expected;
  DummySystemConstraintCalc<double>(*context, &y_expected);
  EXPECT_TRUE(CompareMatrices(y, y_expected, 3 * kEps));

  // Use a lambda function as the selector.
  // This selector's vars = [x(0); x(1); x(2)]
  auto selector2 = [](const auto&, const auto& vars, auto* m_context) {
    m_context->get_mutable_continuous_state_vector().SetFromVector(vars);
  };
  const double p_val = 2;
  context->get_mutable_numeric_parameter(0).SetAtIndex(0, p_val);
  auto constraint2 =
      adapter.Create(system.constraint_index(), *context, selector2, 3);
  const Eigen::Vector3d x_val2(5, 6, 7);
  Eigen::VectorXd y_val2;
  constraint2->Eval(x_val2, &y_val2);
  // Calling constraint2->Eval doesn't change the parameter in the context.
  EXPECT_EQ(context->get_numeric_parameter(0).GetAtIndex(0), p_val);
  selector2(system, x_val2, context.get());
  Eigen::VectorXd y_val2_expected;
  DummySystemConstraintCalc<double>(*context, &y_val2_expected);
  EXPECT_TRUE(CompareMatrices(y_val2, y_val2_expected, 3 * kEps));
}

GTEST_TEST(SystemConstraintAdapter, SolveDummySystemConstraint) {
  DummySystem<double> system;

  SystemConstraintAdapter adapter(&system);

  auto context = system.CreateDefaultContext();

  auto constraint = adapter.Create(system.constraint_index(), *context,
                                   DummySystemSelector1{}, 4);

  solvers::MathematicalProgram prog;
  auto p = prog.NewContinuousVariables<1>();
  auto x = prog.NewContinuousVariables<3>();
  prog.AddConstraint(constraint, {p, x});

  // This initial guess satisfies the constraint already.
  Eigen::VectorXd init_val(4);
  init_val << 2, 3, 4, 35;
  solvers::MathematicalProgramResult result =
      solvers::Solve(prog, init_val, {});
  EXPECT_EQ(result.get_solution_result(),
            solvers::SolutionResult::kSolutionFound);
  const auto p_val = prog.GetSolution(p, result);
  const auto x_val = prog.GetSolution(x, result);
  const double tol = 1E-6;
  EXPECT_GE(p_val(0) * x_val(0) + x_val(1), 2 - tol);
  EXPECT_GE(x_val(2) - x_val(1) * x_val(1) - p_val(0) * x_val(0) * x_val(0),
            -tol);
}

}  // namespace systems
}  // namespace drake
