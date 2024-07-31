#include "drake/systems/primitives/discrete_time_integrator.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/test_utilities/scalar_conversion.h"

namespace drake {
namespace systems {
namespace {

GTEST_TEST(DiscreteTimeIntegratorTest, BasicTest) {
  const double kTimeStep = 0.1;
  DiscreteTimeIntegrator<double> integrator(3, kTimeStep);
  EXPECT_EQ(integrator.get_input_port().size(), 3);
  EXPECT_EQ(integrator.get_output_port().size(), 3);

  EXPECT_EQ(integrator.time_step(), kTimeStep);
  std::optional<PeriodicEventData> data =
      integrator.GetUniquePeriodicDiscreteUpdateAttribute();
  EXPECT_TRUE(data.has_value());
  EXPECT_EQ(data->period_sec(), kTimeStep);

  auto context = integrator.CreateDefaultContext();
  EXPECT_TRUE(CompareMatrices(integrator.get_output_port().Eval(*context),
                              Eigen::Vector3d::Zero(), 1e-14));

  const Eigen::Vector3d x{1.0, 2.0, 3.0};
  integrator.set_integral_value(context.get(), x);
  EXPECT_TRUE(CompareMatrices(x, context->get_discrete_state_vector().value()));
  EXPECT_TRUE(
      CompareMatrices(integrator.get_output_port().Eval(*context), x, 1e-14));

  const Eigen::Vector3d u{4.0, 5.0, 6.0};
  integrator.get_input_port().FixValue(context.get(), u);
  EXPECT_TRUE(CompareMatrices(
      x + kTimeStep * u,
      integrator.EvalUniquePeriodicDiscreteUpdate(*context).value(), 1e-14));
}

GTEST_TEST(DiscreteTimeIntegratorTest, Simulation) {
  const double kTimeStep = 0.1;
  DiscreteTimeIntegrator<double> integrator(2, kTimeStep);

  Simulator<double> simulator(integrator);
  Context<double>& context = simulator.get_mutable_context();

  const Eigen::Vector2d u{1.2, 2.3};
  integrator.get_input_port().FixValue(&context, u);
  EXPECT_TRUE(CompareMatrices(integrator.get_output_port().Eval(context),
                              Eigen::Vector2d::Zero(), 1e-14));

  // From the docs: the output at time `t` is `xₙ` where `n = ceil(t/h)`.
  // Test the output at t = 0.5 * h.
  simulator.AdvanceTo(kTimeStep / 2);
  EXPECT_TRUE(CompareMatrices(integrator.get_output_port().Eval(context),
                              kTimeStep * u, 1e-14));

  // Test the output at t = h.
  simulator.AdvanceTo(kTimeStep);
  EXPECT_TRUE(CompareMatrices(integrator.get_output_port().Eval(context),
                              kTimeStep * u, 1e-14));
}

GTEST_TEST(DiscreteTimeIntegratorTest, ScalarConversion) {
  DiscreteTimeIntegrator<double> integrator(1, 0.1);
  EXPECT_TRUE(is_autodiffxd_convertible(integrator));
  EXPECT_TRUE(is_symbolic_convertible(integrator));
}

}  // namespace
}  // namespace systems
}  // namespace drake
