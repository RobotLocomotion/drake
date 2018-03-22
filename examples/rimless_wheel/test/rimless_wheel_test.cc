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

namespace {

void set_thetadot_to_achieve_energy(const RimlessWheel<double>& rw,
                                    double desired_energy,
                                    systems::Context<double>* context) {
  RimlessWheelState<double>& state = rw.get_mutable_state(context);
  const RimlessWheelParams<double>& params = rw.get_parameters(*context);

  state.set_thetadot(0.0);
  const double energy = rw.CalcTotalEnergy(*context);
  // Note: returns the positive solution.  (-thetadot will have the same
  // energy).
  state.set_thetadot(sqrt(2. * (desired_energy - energy) /
                          (params.mass() * params.length() * params.length())));
}

}  // namespace

// Simulate across one (foot)step and check against analytical solutions.
GTEST_TEST(RimlessWheelTest, StepTest) {
  RimlessWheel<double> rw;

  systems::Simulator<double> simulator(rw);
  auto& context = simulator.get_mutable_context();
  context.set_accuracy(1e-8);

  RimlessWheelState<double>& state = rw.get_mutable_state(&context);
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
  context.set_time(0.0);
  state.set_theta(params.slope() + alpha / 2.0);
  state.set_toe(0.0);
  set_thetadot_to_achieve_energy(rw, steady_state_energy, &context);
  simulator.StepTo(.2);
  // Theta should now be on the other side of zero.
  EXPECT_LT(state.theta(), 0.0);
  // Should have taken one step forward.
  EXPECT_NEAR(state.toe(), step_length, 1e-8);
  // Should still have the same energy (dissipation energy lost = potential
  // energy gained).
  EXPECT_NEAR(rw.CalcTotalEnergy(context), steady_state_energy, 1e-6);

  // Walking too fast should lose energy through impact.
  context.set_time(0.0);
  state.set_theta(params.slope() + alpha / 2.0);
  state.set_toe(0.0);
  set_thetadot_to_achieve_energy(rw, steady_state_energy, &context);
  state.set_thetadot(state.thetadot() + 0.2);
  double initial_energy = rw.CalcTotalEnergy(context);
  simulator.StepTo(.2);
  // Theta should now be on the other side of zero.
  EXPECT_LT(state.theta(), 0.0);
  // Should have taken one step forward.
  EXPECT_NEAR(state.toe(), step_length, 1e-8);
  // Should have lost total energy on collision.
  EXPECT_LT(rw.CalcTotalEnergy(context), initial_energy-0.01);

  // Walking too slow should gain energy through impact.
  context.set_time(0.0);
  state.set_theta(params.slope() + alpha / 2.0);
  state.set_toe(0.0);
  set_thetadot_to_achieve_energy(rw, steady_state_energy, &context);
  state.set_thetadot(state.thetadot() - 0.2);
  initial_energy = rw.CalcTotalEnergy(context);
  simulator.StepTo(.2);
  // Theta should now be on the other side of zero.
  EXPECT_LT(state.theta(), 0.);
  // Should have taken one step forward.
  EXPECT_NEAR(state.toe(), step_length, 1e-8);
  // Should have gained total energy on collision.
  EXPECT_GT(rw.CalcTotalEnergy(context), initial_energy+0.01);

  // Rolling uphill should always lose energy.
  context.set_time(0.0);
  // Leaning uphill.
  state.set_theta(params.slope() - alpha/2.0);
  state.set_thetadot(-4.);
  state.set_toe(0.0);
  initial_energy = rw.CalcTotalEnergy(context);
  simulator.StepTo(.2);
  // Theta should now be on the other side of zero.
  EXPECT_GT(state.theta(), 0.);
  // Should have taken one step backward.
  EXPECT_NEAR(state.toe(), -step_length, 1e-8);
  // Should have gained total energy on collision.
  EXPECT_LT(rw.CalcTotalEnergy(context), initial_energy+0.1);
}

}  // namespace
}  // namespace rimless_wheel
}  // namespace examples
}  // namespace drake
