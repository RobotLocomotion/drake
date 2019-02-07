#include "drake/systems/optimization/system_constraint_adapter.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/math/autodiff_gradient.h"
#include "drake/solvers/solve.h"
#include "drake/systems/optimization/test/system_optimization_test_util.h"
#include "drake/systems/primitives/linear_system.h"

namespace drake {
namespace systems {
const double kInf = std::numeric_limits<double>::infinity();
const double kEps = std::numeric_limits<double>::epsilon();

// Assumes that vars = [p; x(0); x(1); x(2)].
struct DummySystemUpdater1 {
  template <typename T>
  void operator()(const System<T>&, const Eigen::Ref<const VectorX<T>>& vars,
                  Context<T>* context) {
    context->SetContinuousState(vars.template tail<3>());
    context->get_mutable_numeric_parameter(0).SetAtIndex(0, vars(0));
  }
};

GTEST_TEST(SystemConstraintAdapterTest, CreateSystemConstraintWrapper) {
  DummySystem<double> system;

  SystemConstraintAdapter adapter(&system);

  auto context = system.CreateDefaultContext();

  auto constraint1 = adapter.Create(system.constraint_index(), *context,
                                    DummySystemUpdater1{}, 4);
  EXPECT_TRUE(
      CompareMatrices(constraint1->lower_bound(), Eigen::Vector2d(2, 0)));
  EXPECT_TRUE(
      CompareMatrices(constraint1->upper_bound(), Eigen::Vector2d(kInf, kInf)));

  EXPECT_EQ(constraint1->get_description(), "dummy_system_constraint");

  const Eigen::Vector4d var(2, 3, 4, 5);
  Eigen::VectorXd y;
  constraint1->Eval(var, &y);
  // Compute the constraint by hand, and compare it against y.
  EXPECT_TRUE(CompareMatrices(y, Eigen::Vector2d(10, -29), 10 * kEps));

  DummySystemUpdater1 selector;
  selector.operator()<double>(system, var, context.get());
  Eigen::VectorXd y_expected;
  DummySystemConstraintCalc<double>(*context, &y_expected);
  EXPECT_TRUE(CompareMatrices(y, y_expected, 3 * kEps));

  // Use a lambda function as the selector.
  // This selector's vars = [x(0); x(1); x(2)]
  // We use "auto" here to allow transmogrify the scalar types.
  auto selector2 = [](const auto&, const auto& vars, auto* m_context) {
    m_context->SetContinuousState(vars);
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

GTEST_TEST(SystemConstraintAdapterTest, SolveDummySystemConstraint) {
  DummySystem<double> system;

  SystemConstraintAdapter adapter(&system);

  auto context = system.CreateDefaultContext();

  auto constraint = adapter.Create(system.constraint_index(), *context,
                                   DummySystemUpdater1{}, 4);

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
  const auto p_val = result.GetSolution(p);
  const auto x_val = result.GetSolution(x);
  const double tol = 1E-6;
  EXPECT_GE(p_val(0) * x_val(0) + x_val(1), 2 - tol);
  EXPECT_GE(x_val(2) - x_val(1) * x_val(1) - p_val(0) * x_val(0) * x_val(0),
            -tol);
}

GTEST_TEST(SystemConstraintAdapterTest, ExternalSystemConstraint) {
  Eigen::Matrix2d A;
  A << 0, 1, 0, 0;
  LinearSystem<double> double_integrator(A, Eigen::Vector2d(0, 1),
                                         Eigen::Matrix2d::Identity(),
                                         Eigen::Vector2d::Zero());

  // Now add an external system constraint that we don't want the velocity to be
  // to high.
  ExternalSystemConstraint velocity_bound =
      ExternalSystemConstraint::MakeForAllScalars(
          "|velocity| < 5", SystemConstraintBounds(Vector1d(-5), Vector1d(5)),
          [](const auto& system, const auto& context, auto* value) {
            *value = context.get_continuous_state_vector()
                         .CopyToVector()
                         .template tail<1>();
          });
  const SystemConstraintIndex velocity_bound_index =
      double_integrator.AddExternalConstraint(velocity_bound);

  SystemConstraintAdapter adapter(&double_integrator);

  auto context = double_integrator.CreateDefaultContext();

  auto selector = [](const auto&, const auto& vars, auto* m_context) {
    m_context->SetContinuousState(vars);
  };

  auto constraint = adapter.Create(velocity_bound_index, *context, selector, 2);

  solvers::MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<2>();
  prog.AddConstraint(constraint, x);

  // The initial guess for velocity is 6, ouside of the bound.
  auto result = Solve(prog, Eigen::Vector2d(1, 6));

  EXPECT_EQ(result.get_solution_result(),
            solvers::SolutionResult::kSolutionFound);
  const double tol{1E-6};
  EXPECT_LE(result.GetSolution(x(1)), 5 + tol);
  EXPECT_GE(result.GetSolution(x(1)), -5 - tol);
}

void CheckBoundingBoxConstraint(
    const solvers::Binding<solvers::Constraint>& constraint,
    const symbolic::Variable& var, double lower, double upper,
    double tol = 1E-14) {
  const auto bounding_box =
      solvers::internal::BindingDynamicCast<solvers::BoundingBoxConstraint>(
          constraint);
  EXPECT_EQ(bounding_box.variables().size(), 1);
  EXPECT_EQ(bounding_box.variables()(0), var);
  EXPECT_NEAR(bounding_box.evaluator()->lower_bound()(0), lower, tol);
  EXPECT_NEAR(bounding_box.evaluator()->upper_bound()(0), upper, tol);
}

void CheckLinearConstraint(
    const solvers::Binding<solvers::Constraint>& constraint,
    const Eigen::Ref<const VectorX<symbolic::Variable>>& vars,
    const Eigen::Ref<const Eigen::MatrixXd>& A,
    const Eigen::Ref<const Eigen::VectorXd>& lower,
    const Eigen::Ref<const Eigen::VectorXd>& upper, double tol = 1E-14) {
  const auto linear_constraint =
      solvers::internal::BindingDynamicCast<solvers::LinearConstraint>(
          constraint);
  EXPECT_EQ(linear_constraint.variables(), vars);
  EXPECT_TRUE(CompareMatrices(linear_constraint.evaluator()->A(), A, tol));
  EXPECT_TRUE(CompareMatrices(linear_constraint.evaluator()->lower_bound(),
                              lower, tol));
  EXPECT_TRUE(CompareMatrices(linear_constraint.evaluator()->upper_bound(),
                              upper, tol));
}

void CheckLinearEqualityConstraint(
    const solvers::Binding<solvers::Constraint>& constraint,
    const Eigen::Ref<const VectorX<symbolic::Variable>>& vars,
    const Eigen::Ref<const Eigen::MatrixXd>& A,
    const Eigen::Ref<const Eigen::VectorXd>& rhs, double tol = 1E-14) {
  const auto linear_eq_constraint =
      solvers::internal::BindingDynamicCast<solvers::LinearEqualityConstraint>(
          constraint);
  EXPECT_EQ(linear_eq_constraint.variables(), vars);
  EXPECT_TRUE(CompareMatrices(linear_eq_constraint.evaluator()->A(), A, tol));
  EXPECT_TRUE(CompareMatrices(linear_eq_constraint.evaluator()->lower_bound(),
                              rhs, tol));
}

GTEST_TEST(SystemConstraintAdapterTest, MaybeCreateConstraintSymbolically1) {
  Eigen::Matrix2d A;
  A << 0, 1, 0, 0;
  LinearSystem<double> double_integrator(A, Eigen::Vector2d(0, 1),
                                         Eigen::Matrix2d::Identity(),
                                         Eigen::Vector2d::Zero());

  // Create an arbitrary external constraint with linear forms.
  // 0 <= x(0) + 1 <= 5
  // 1 <= 2 * x(0) + x(1) <= 4
  // x(0) + 2 * x(1) = 3
  ExternalSystemConstraint system_constraint =
      ExternalSystemConstraint::MakeForAllScalars(
          "constraint", {Eigen::Vector3d(0, 1, 3), Eigen::Vector3d(5, 4, 3)},
          [](const auto& system, const auto& context, auto* value) {
            auto x = context.get_continuous_state_vector().CopyToVector();
            (*value)(0) = x(0) + 1;
            (*value)(1) = 2 * x(0) + x(1);
            (*value)(2) = x(0) + 2 * x(1);
          });

  const SystemConstraintIndex system_constraint_index =
      double_integrator.AddExternalConstraint(system_constraint);

  SystemConstraintAdapter adapter(&double_integrator);

  auto context_symbolic = adapter.system_symbolic().CreateDefaultContext();

  const symbolic::Variable a("a");
  const symbolic::Variable b("b");

  context_symbolic->get_mutable_continuous_state_vector().SetFromVector(
      Vector2<symbolic::Expression>(a, b));

  auto constraints = adapter.MaybeCreateConstraintSymbolically(
      system_constraint_index, *context_symbolic);
  EXPECT_TRUE(constraints.has_value());
  ASSERT_EQ(constraints->size(), 3);
  CheckBoundingBoxConstraint(constraints.value()[0], a, -1, 4);
  CheckLinearConstraint(constraints.value()[1],
                        Vector2<symbolic::Variable>(a, b),
                        Eigen::RowVector2d(2, 1), Vector1d(1), Vector1d(4));
  CheckLinearEqualityConstraint(constraints.value()[2],
                                Vector2<symbolic::Variable>(a, b),
                                Eigen::RowVector2d(1, 2), Vector1d(3));

  // Now test context.x = [a + b; 1]. Namely it contains both expression
  // and constant.
  context_symbolic->get_mutable_continuous_state().SetFromVector(
      Vector2<symbolic::Expression>(a + b, 1));
  // The newly generated constraint should be
  // -1 <= a + b <= 4
  // 0 <= 2 * a + 2 * b <= 3
  // a + b = 1
  constraints = adapter.MaybeCreateConstraintSymbolically(
      system_constraint_index, *context_symbolic);
  EXPECT_TRUE(constraints.has_value());
  EXPECT_EQ(constraints->size(), 3);
  CheckLinearConstraint(constraints.value()[0],
                        Vector2<symbolic::Variable>(a, b),
                        Eigen::RowVector2d(1, 1), Vector1d(-1), Vector1d(4));
  CheckLinearConstraint(constraints.value()[1],
                        Vector2<symbolic::Variable>(a, b),
                        Eigen::RowVector2d(2, 2), Vector1d(0), Vector1d(3));
  CheckLinearEqualityConstraint(constraints.value()[2],
                                Vector2<symbolic::Variable>(a, b),
                                Eigen::RowVector2d(1, 1), Vector1d(1));

  // Now test a new context with context.x = [a^2, 1]. Hence the constraints
  // are nonlinear
  context_symbolic->get_mutable_continuous_state().SetFromVector(
      Vector2<symbolic::Expression>(a * a, 1));
  constraints = adapter.MaybeCreateConstraintSymbolically(
      system_constraint_index, *context_symbolic);
  EXPECT_FALSE(constraints.has_value());
}
}  // namespace systems
}  // namespace drake
