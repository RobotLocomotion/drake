#include "drake/examples/rimless_wheel/rimless_wheel.h"

#include <gtest/gtest.h>

#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/test_utilities/scalar_conversion.h"

using std::cos;
using std::sin;
using std::sqrt;

namespace drake {
namespace examples {
namespace rimless_wheel {
namespace {

GTEST_TEST(RimlessWheelTest, ToAutoDiff) {
  RimlessWheel<double> rw;
  EXPECT_TRUE(is_autodiffxd_convertible(rw));
}

// Given the desired energy and theta, compute thetadot.
void set_thetadot_to_achieve_energy(const RimlessWheel<double>& rw,
                                    double desired_energy,
                                    systems::Context<double>* context) {
  RimlessWheelContinuousState<double>& state =
      rw.get_mutable_continuous_state(context);
  const RimlessWheelParams<double>& params = rw.get_parameters(*context);

  state.set_thetadot(0.0);
  const double energy = rw.CalcTotalEnergy(*context);
  // Note: returns the positive solution.  (-thetadot will have the same
  // energy).
  state.set_thetadot(sqrt(2. * (desired_energy - energy) /
                          (params.mass() * params.length() * params.length())));
}

// Simulate across one (foot)step and check against analytical solutions.
GTEST_TEST(RimlessWheelTest, StepTest) {
  const RimlessWheel<double> rw;

  systems::Simulator<double> simulator(rw);
  auto& context = simulator.get_mutable_context();
  context.SetAccuracy(1e-8);

  RimlessWheelContinuousState<double>& state =
      rw.get_mutable_continuous_state(&context);
  double& toe = rw.get_mutable_toe_position(&context.get_mutable_state());
  bool& double_support =
      rw.get_mutable_double_support(&context.get_mutable_state());
  const RimlessWheelParams<double>& params = rw.get_parameters(context);
  const double alpha = rw.calc_alpha(params);

  // From the course notes, the rolling fixed point is at:
  state.set_theta(params.slope() - alpha);
  state.set_thetadot(cos(2 * alpha) / sin(2 * alpha) *
                     sqrt(4 * params.gravity() / params.length() * sin(alpha) *
                          sin(params.slope())));
  const double steady_state_energy = rw.CalcTotalEnergy(context);

  const double step_length = 2 * params.length() * sin(alpha);

  // A point on that limit cycle just before a forward step:
  context.SetTime(0.0);
  state.set_theta(params.slope() + alpha / 2.0);
  toe = 0.0;
  double_support = false;
  set_thetadot_to_achieve_energy(rw, steady_state_energy, &context);
  simulator.AdvanceTo(.2);
  // Theta should now be on the other side of zero.
  EXPECT_LT(state.theta(), 0.0);
  // Should have taken one step forward.
  EXPECT_NEAR(toe, step_length, 1e-8);
  // Should still have the same energy (dissipation energy lost = potential
  // energy gained). The error tolerance below seems to work well.
  EXPECT_NEAR(rw.CalcTotalEnergy(context), steady_state_energy, 1e-5);

  // Lose at least this much energy (chosen as an arbitrary small positive
  // number).
  const double threshold = 0.01;

  // Walking too fast should lose energy through impact.
  context.SetTime(0.0);
  state.set_theta(params.slope() + alpha / 2.0);
  toe = 0.0;
  double_support = false;
  set_thetadot_to_achieve_energy(rw, steady_state_energy, &context);
  state.set_thetadot(state.thetadot() + 0.2);
  double initial_energy = rw.CalcTotalEnergy(context);
  simulator.Initialize();
  simulator.AdvanceTo(.2);
  // Theta should now be on the other side of zero.
  EXPECT_LT(state.theta(), 0.0);
  // Should have taken one step forward.
  EXPECT_NEAR(toe, step_length, 1e-8);
  // Should have lost total energy on collision.
  EXPECT_LT(rw.CalcTotalEnergy(context), initial_energy - threshold);

  // Walking too slow should gain energy through impact.
  context.SetTime(0.0);
  state.set_theta(params.slope() + alpha / 2.0);
  toe = 0.0;
  double_support = false;
  set_thetadot_to_achieve_energy(rw, steady_state_energy, &context);
  state.set_thetadot(state.thetadot() - 0.2);
  initial_energy = rw.CalcTotalEnergy(context);
  simulator.Initialize();
  simulator.AdvanceTo(.2);
  // Theta should now be on the other side of zero.
  EXPECT_LT(state.theta(), 0.);
  // Should have taken one step forward.
  EXPECT_NEAR(toe, step_length, 1e-8);
  // Should have gained total energy on collision.
  EXPECT_GT(rw.CalcTotalEnergy(context), initial_energy + threshold);

  // Rolling uphill should always lose energy.
  context.SetTime(0.0);
  // Leaning uphill.
  state.set_theta(params.slope() - alpha / 2.0);
  state.set_thetadot(-4.);
  toe = 0.0;
  double_support = false;
  simulator.Initialize();
  initial_energy = rw.CalcTotalEnergy(context);
  simulator.AdvanceTo(.2);
  // Theta should now be on the other side of zero.
  EXPECT_GT(state.theta(), 0.);
  // Should have taken one step backward.
  EXPECT_NEAR(toe, -step_length, 1e-8);
  // Should have gained total energy on collision.
  EXPECT_LT(rw.CalcTotalEnergy(context), initial_energy + 0.1);
}

// Check that the double-support logic makes the standing fixed point a fixed
// point.
GTEST_TEST(RimlessWheelTest, FixedPointTest) {
  const RimlessWheel<double> rw;

  systems::Simulator<double> simulator(rw);
  auto& context = simulator.get_mutable_context();
  context.SetAccuracy(1e-8);

  RimlessWheelContinuousState<double>& state =
      rw.get_mutable_continuous_state(&context);
  double& toe = rw.get_mutable_toe_position(&context.get_mutable_state());
  bool& double_support =
      rw.get_mutable_double_support(&context.get_mutable_state());
  const RimlessWheelParams<double>& params = rw.get_parameters(context);
  const double alpha = rw.calc_alpha(params);

  // Start the foot just above the ground (in each direction).  Chosen as
  // positive number that is sufficiently small to trigger double support on
  // first collision.
  const double angle_above_touchdown = 1e-5;

  // Front foot down.
  context.SetTime(0.0);
  state.set_theta(params.slope() + alpha - angle_above_touchdown);
  state.set_thetadot(0.0);
  toe = 0.0;
  double_support = false;
  simulator.AdvanceTo(0.2);
  EXPECT_TRUE(double_support);
  EXPECT_NEAR(std::abs(state.theta() - params.slope()), alpha, 1e-8);
  EXPECT_EQ(state.thetadot(), 0.0);

  // Back foot down.
  context.SetTime(0.0);
  state.set_theta(params.slope() - alpha + angle_above_touchdown);
  state.set_thetadot(0.0);
  toe = 0.0;
  double_support = false;
  simulator.Initialize();
  simulator.AdvanceTo(0.2);
  EXPECT_TRUE(double_support);
  EXPECT_NEAR(std::abs(state.theta() - params.slope()), alpha, 1e-8);
  EXPECT_EQ(state.thetadot(), 0.0);
}

// Use the integrator's dense output to check the details of integration.
GTEST_TEST(RimlessWheelTest, DenseOutput) {
  const RimlessWheel<double> rw;
  systems::Simulator<double> simulator(rw);
  auto& context = simulator.get_mutable_context();
  context.SetAccuracy(1e-8);
  auto& integrator = simulator.get_mutable_integrator();

  RimlessWheelContinuousState<double>& state =
      rw.get_mutable_continuous_state(&context);
  double& toe = rw.get_mutable_toe_position(&context.get_mutable_state());
  const RimlessWheelParams<double>& params = rw.get_parameters(context);
  const double alpha = rw.calc_alpha(params);

  // A point that is just before a forward step:
  state.set_theta(params.slope() + alpha / 2.0);
  state.set_thetadot(1.);
  toe = 0.0;

  integrator.StartDenseIntegration();
  simulator.AdvanceTo(.2);
  const std::unique_ptr<trajectories::PiecewisePolynomial<double>>
      dense_output = integrator.StopDenseIntegration();

  // Check that I indeed took a step.
  EXPECT_GE(toe, 0.0);

  // Ensure that I can find the discontinuity in the dense output.
  int num_forward_steps = 0;
  const double kEpsilon = std::numeric_limits<double>::epsilon();
  for (const double time : dense_output->get_segment_times()) {
    // We have theta > 0 before impact, and theta < 0 after the impact.
    if (dense_output->value(time - kEpsilon)(0) > 0 &&
        dense_output->value(time + kEpsilon)(0) < 0) {
      num_forward_steps++;
    }
  }
  EXPECT_EQ(num_forward_steps, 1);
}

}  // namespace
}  // namespace rimless_wheel
}  // namespace examples
}  // namespace drake
