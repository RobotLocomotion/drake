#include "drake/systems/lcm/lcm_driven_loop.h"

#include "bot_core/robot_state_t.hpp"

namespace drake {
namespace systems {

void test() {
  drake::lcm::DrakeLcm lcm;

  auto sys = systems::lcm::LcmSubscriberSystem::Make<bot_core::robot_state_t>(
      "EST_ROBOT_STATE", &lcm);
  auto msg_to_time = std::make_unique<
      systems::lcm::UtimeMessageToSeconds<bot_core::robot_state_t>>();

  lcm::LcmDrivenLoop dut(&lcm, *sys, nullptr, sys.get(),
                         std::move(msg_to_time));

  dut.RunWithDefaultInitialization();
}

}  // namespace systems
}  // namespace drake

int main() {
  drake::systems::test();

  return 0;
}
