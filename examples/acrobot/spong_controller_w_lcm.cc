/*
 * An acrobot Spong controller that communicates to run_plant_w_lcm
 * through LCM, implemented by the following diagram system:
 *
 * LcmSubscriberSystem—>
 * AcrobotStateReceiver—>
 * AcrobotSpongController—>
 * AcrobotCommandSender—>
 * LcmPublisherSystem
 *
 */
#include <chrono>
#include <memory>

#include <gflags/gflags.h>

#include "drake/examples/acrobot/acrobot_lcm.h"
#include "drake/examples/acrobot/spong_controller.h"
#include "drake/lcmt_acrobot_u.hpp"
#include "drake/lcmt_acrobot_x.hpp"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/lcm/lcm_interface_system.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"

DEFINE_double(time_limit_sec, std::numeric_limits<double>::infinity(),
              "Number of seconds to run (default: infinity).");

namespace drake {
namespace examples {
namespace acrobot {
namespace {

int DoMain() {
  drake::systems::DiagramBuilder<double> builder;
  const std::string channel_x = "acrobot_xhat";
  const std::string channel_u = "acrobot_u";

  // -----------------controller--------------------------------------
  // Create state receiver.
  auto lcm = builder.AddSystem<systems::lcm::LcmInterfaceSystem>();
  auto state_sub = builder.AddSystem(
      systems::lcm::LcmSubscriberSystem::Make<lcmt_acrobot_x>(channel_x, lcm));
  auto state_receiver = builder.AddSystem<AcrobotStateReceiver>();
  builder.Connect(state_sub->get_output_port(),
                  state_receiver->get_input_port(0));

  // Create command sender.
  auto command_pub = builder.AddSystem(
      systems::lcm::LcmPublisherSystem::Make<lcmt_acrobot_u>(
        channel_u, lcm, 0.001));
  auto command_sender = builder.AddSystem<AcrobotCommandSender>();
  builder.Connect(command_sender->get_output_port(0),
                  command_pub->get_input_port());

  auto controller = builder.AddSystem<AcrobotSpongController>();
  builder.Connect(controller->get_output_port(0),
                  command_sender->get_input_port(0));
  builder.Connect(state_receiver->get_output_port(0),
                  controller->get_input_port(0));

  auto diagram = builder.Build();

  systems::Simulator<double> simulator(*diagram);

  simulator.set_target_realtime_rate(1.0);
  simulator.Initialize();
  simulator.AdvanceTo(FLAGS_time_limit_sec);

  return 0;
}

}  // namespace
}  // namespace acrobot
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return drake::examples::acrobot::DoMain();
}
