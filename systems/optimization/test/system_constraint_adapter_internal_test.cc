#include "drake/systems/optimization/system_constraint_adapter_internal.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/math/autodiff_gradient.h"
#include "drake/systems/optimization/test/system_optimization_test_util.h"
#include "drake/systems/primitives/linear_system.h"

namespace drake {
namespace systems {
namespace internal {
GTEST_TEST(UpdateContextForSymbolicSystemConstraintTest, TestDummySystem) {
  DummySystem<symbolic::Expression> system_symbolic;
  DummySystem<AutoDiffXd> system_autodiff;
  DummySystem<double> system_double;
  auto context_symbolic = system_symbolic.CreateDefaultContext();
  auto context_autodiff = system_autodiff.CreateDefaultContext();
  auto context_double = system_double.CreateDefaultContext();

  const symbolic::Variable a("a");
  const symbolic::Variable b("b");
  const symbolic::Variable t("t");

  context_symbolic->get_mutable_continuous_state_vector().GetAtIndex(0) = a;
  context_symbolic->get_mutable_continuous_state_vector().GetAtIndex(1) = 1;
  context_symbolic->get_mutable_numeric_parameter(0).SetAtIndex(0, b);
  context_symbolic->set_time(t);

  UpdateContextForSymbolicSystemConstraint updater(context_symbolic.get());
  EXPECT_EQ(updater.bound_variables(), Vector3<symbolic::Variable>(a, t, b));

  const double a_val = 2;
  const double b_val = 4;
  const double t_val = 3;
  Eigen::Vector3d var_val(a_val, t_val, b_val);
  updater(system_double, var_val, context_double.get());
  EXPECT_EQ(context_double->get_continuous_state_vector().GetAtIndex(0), a_val);
  EXPECT_EQ(context_double->get_continuous_state_vector().GetAtIndex(1), 1);
  EXPECT_EQ(context_double->get_numeric_parameter(0).GetAtIndex(0), b_val);
  EXPECT_EQ(context_double->get_time(), t_val);

  AutoDiffVecXd var_autodiff = math::initializeAutoDiffGivenGradientMatrix(
      Eigen::VectorXd(var_val), Eigen::MatrixXd::Identity(3, 3));
  updater(system_autodiff, var_autodiff, context_autodiff.get());
  auto compare_autodiffxd = [](const AutoDiffXd& v,
                               const AutoDiffXd& v_expected, double tol) {
    EXPECT_NEAR(v.value(), v_expected.value(), tol);
    EXPECT_TRUE(
        CompareMatrices(v.derivatives(), v_expected.derivatives(), tol));
  };
  compare_autodiffxd(
      context_autodiff->get_continuous_state_vector().GetAtIndex(0),
      var_autodiff(0), 0);
  EXPECT_EQ(
      context_autodiff->get_continuous_state_vector().GetAtIndex(1).value(), 1);
  EXPECT_EQ(context_autodiff->get_continuous_state_vector()
                .GetAtIndex(1)
                .derivatives()
                .size(),
            0);
  compare_autodiffxd(context_autodiff->get_numeric_parameter(0).GetAtIndex(0),
                     var_autodiff(2), 0);
  compare_autodiffxd(context_autodiff->get_time(), var_autodiff(1), 0);
}
}  // namespace internal
}  // namespace systems
}  // namespace drake
