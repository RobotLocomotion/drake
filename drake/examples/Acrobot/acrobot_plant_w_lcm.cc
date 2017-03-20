/*
 * A simulated acrobot plant that communicates to its controller through LCM,
 * implemented by the following Diagram system:
 *
 * LcmSubscriberSystem -->
 * AcrobotCommandReceiver -->
 * AcrobotPlant -->
 * AcrobotStateSender -->
 * LcmPublisherSystem
 *
 */
#include "drake/examples/Acrobot/acrobot_plant.h"

#include <memory>

#include <gflags/gflags.h>

#include "drake/common/drake_path.h"
#include "drake/examples/Acrobot/acrobot_lcm.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/lcmt_acrobot_u.hpp"
#include "drake/lcmt_acrobot_x.hpp"
#include "drake/multibody/joints/floating_base_types.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_plant/drake_visualizer.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"

DEFINE_double(simulation_sec, std::numeric_limits<double>::infinity(),
              "Number of seconds to simulate.");
DEFINE_double(realtime_factor, 1.0,
              "Playback speed.  See documentation for "
              "Simulator::set_target_realtime_rate() for details.");

using std::chrono::milliseconds;
using std::this_thread::sleep_for;

namespace drake {
namespace examples {
namespace acrobot {
namespace {

int DoMain() {
  drake::systems::DiagramBuilder<double> builder;
  const std::string channel_x = "acrobot_xhat";
  const std::string channel_u = "acrobot_u";

  lcm::DrakeLcm lcm;
  auto tree = std::make_unique<RigidBodyTree<double>>();
  parsers::urdf::AddModelInstanceFromUrdfFileToWorld(
      GetDrakePath() + "/examples/Acrobot/Acrobot.urdf",
      multibody::joints::kFixed, tree.get());
  auto publisher = builder.AddSystem<systems::DrakeVisualizer>(*tree, &lcm);
  auto acrobot = builder.AddSystem<AcrobotPlant>();
  // Connects plant to visualizer.
  builder.Connect(acrobot->get_output_port(0), publisher->get_input_port(0));

  // Creates command receiver and subscriber.
  auto command_sub = builder.AddSystem(
      systems::lcm::LcmSubscriberSystem::Make<lcmt_acrobot_u>(channel_u, &lcm));
  auto command_receiver = builder.AddSystem<AcrobotCommandReceiver>();
  builder.Connect(command_sub->get_output_port(0),
                  command_receiver->get_input_port(0));

  // Creates state sender and publisher.
  auto state_pub = builder.AddSystem(
      systems::lcm::LcmPublisherSystem::Make<lcmt_acrobot_x>(channel_x, &lcm));
  auto state_sender = builder.AddSystem<AcrobotStateSender>();
  builder.Connect(state_sender->get_output_port(0),
                  state_pub->get_input_port(0));

  // Connects plant to command receiver and state sender.
  builder.Connect(command_receiver->get_output_port(0),
                  acrobot->get_input_port(0));
  builder.Connect(acrobot->get_output_port(0), state_sender->get_input_port(0));

  auto diagram = builder.Build();
  systems::Simulator<double> simulator(*diagram);

  // Sets an initial condition near the stable fixed point.
  systems::Context<double>* acrobot_context =
      diagram->GetMutableSubsystemContext(simulator.get_mutable_context(),
                                          acrobot);
  AcrobotStateVector<double>* x0 = dynamic_cast<AcrobotStateVector<double>*>(
      acrobot_context->get_mutable_continuous_state_vector());
  DRAKE_DEMAND(x0 != nullptr);
  x0->set_theta1(0.1);
  x0->set_theta2(0.1);
  x0->set_theta1dot(0.0);
  x0->set_theta2dot(0.0);

  lcm.StartReceiveThread();

  simulator.set_target_realtime_rate(FLAGS_realtime_factor);
  simulator.Initialize();
  // Simulate for a very long time.
  simulator.StepTo(FLAGS_simulation_sec);

  return 0;
}
}  // namespace
}  // namespace acrobot
}  // namespace examples
}  // namespace drake

int main() { return drake::examples::acrobot::DoMain(); }
