#include "sim/common/simulator_config_functions.h"

#include <gtest/gtest.h>

#include "drake/systems/framework/leaf_system.h"

namespace anzu {
namespace sim {
namespace {

// A hollow shell of a System.
class DummySystem final : public drake::systems::LeafSystem<double> {
 public:
  DummySystem() {}
};

GTEST_TEST(SimulatorConfigFunctionsTest, ExtractDefaultsTest) {
  const DummySystem dummy;
  drake::systems::Simulator<double> simulator(dummy);

  // We can always extract the defaults.
  const SimulatorConfig defaults = ExtractSimulatorConfig(simulator);
  EXPECT_EQ(defaults.integration_scheme, "runge_kutta3");
  // TODO(jeremy.nimmer) Drake returns incorrect defaults for max_step_size and
  // accuracy.  Once Drake is fixed, we should EXPECT_EQ instead here.
  EXPECT_GT(defaults.max_step_size, 0);
  EXPECT_GT(defaults.accuracy, 0);
  EXPECT_EQ(defaults.use_error_control, true);
  EXPECT_EQ(defaults.target_realtime_rate, 0.0);
  EXPECT_EQ(defaults.publish_every_time_step, false);
}

GTEST_TEST(SimulatorConfigFunctionsTest, RoundTripTest) {
  SimulatorConfig bespoke;
  bespoke.integration_scheme = "runge_kutta5";
  bespoke.max_step_size = 3.0e-3;
  bespoke.accuracy = 3.0e-2;
  bespoke.use_error_control = true;
  bespoke.target_realtime_rate = 3.0;
  bespoke.publish_every_time_step = true;

  const DummySystem dummy;
  drake::systems::Simulator<double> simulator(dummy);
  ApplySimulatorConfig(&simulator, bespoke);

  const SimulatorConfig readback = ExtractSimulatorConfig(simulator);
  EXPECT_EQ(readback.integration_scheme, bespoke.integration_scheme);
  EXPECT_EQ(readback.max_step_size, bespoke.max_step_size);
  EXPECT_EQ(readback.accuracy, bespoke.accuracy);
  EXPECT_EQ(readback.use_error_control, bespoke.use_error_control);
  EXPECT_EQ(readback.target_realtime_rate, bespoke.target_realtime_rate);
  EXPECT_EQ(readback.publish_every_time_step, bespoke.publish_every_time_step);
}

}  // namespace
}  // namespace sim
}  // namespace anzu
