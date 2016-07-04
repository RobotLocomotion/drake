#include "gtest/gtest.h"

// TODO(liang.fok) Automatically generate this file.

#include "drake/examples/spring_mass/lcm_input_system.h"
#include "drake/examples/spring_mass/spring_mass_lcm_vector.h"

namespace drake {
namespace examples {
namespace spring_mass {
namespace {

using drake::systems::LCMInputSystem;

// Tests the default values in
// drake::examples::spring_mass::SpringMassLCMVector.
GTEST_TEST(LCMInputSystemTest, InstantiateTest) {
  // Instantiates an LCMInputSystem and verifies that it was successfully stored
  // in a unique_ptr. The variable is called "dut" to indicate it is the
  // "device under test".
  std::unique_ptr<LCMInputSystem<double, SpringMassLCMVector<double>,
    lcmt_spring_mass_state_t>> dut(
      new LCMInputSystem<double, SpringMassLCMVector<double>,
      lcmt_spring_mass_state_t>("test_channel_name"));

  EXPECT_NE(dut.get(), nullptr);
}

}  // namespace
}  // namespace spring_mass
}  // namespace examples
}  // namespace drake
