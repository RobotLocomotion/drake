#include <memory>

#include <gflags/gflags.h>

#include "drake/examples/hsr/hsr_world.h"
#include "drake/examples/hsr/parameters/sim_parameters.h"
#include "drake/examples/hsr/sim_command_sender.h"
#include "drake/examples/hsr/sim_status_receiver.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/lcm/lcm_interface_system.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"
#include "drake/systems/primitives/constant_vector_source.h"

namespace drake {
namespace examples {
namespace hsr {
namespace {

int DoMain() {
  const auto& sim_parameters = hsr::parameters::hsr_sim_flags();

  const HsrWorld<double> hsr_world("");
  const auto& hsr_plant = hsr_world.get_hsr_plant("hsr");

  // Build a generic multibody plant.
  systems::DiagramBuilder<double> builder;
  auto lcm = builder.AddSystem<systems::lcm::LcmInterfaceSystem>();

  // // HSR status subscriber.
  // auto hsr_status_subscriber = builder.AddSystem(
  //     systems::lcm::LcmSubscriberSystem::Make<drake::lcmt_hsr_sim_status>(
  //         "hsr_sim_status_channel", lcm));
  // auto hsr_status_receiver =
  //     builder.AddSystem<hsr::SimStatusReceiver>(&hsr_plant);
  // builder.Connect(hsr_status_subscriber->get_output_port(),
  //                 hsr_status_receiver->get_sim_status_input_port());

  // Create a constant resource to publish as the command.
  drake::VectorX<double> constant_pos_value = drake::VectorX<double>::Zero(
      hsr_plant.num_positions() + hsr_plant.num_velocities());
  // Set the w of the quaternion base to be 1.0 to make the state to be valid.
  constant_pos_value(0) = 1.0;
  const std::string joint_name = "head_tilt_joint";
  constant_pos_value[hsr_plant.GetJointByName(joint_name).position_start()] = 1;

  auto desired_pos_constant_source =
      builder.template AddSystem<systems::ConstantVectorSource<double>>(
          constant_pos_value);
  desired_pos_constant_source->set_name("desired_pos_constant_source");
  // HSR command publisher.
  auto hsr_command_publisher = builder.AddSystem(
      systems::lcm::LcmPublisherSystem::Make<drake::lcmt_hsr_sim_command>(
          "hsr_sim_command_channel", lcm, 0.005 /* publish period */));
  auto hsr_command_sender =
      builder.AddSystem<hsr::SimCommandSender>(&hsr_plant);
  builder.Connect(desired_pos_constant_source->get_output_port(),
                  hsr_command_sender->get_desired_state_input_port());
  builder.Connect(hsr_command_sender->get_sim_command_output_port(),
                  hsr_command_publisher->get_input_port());

  auto diagram = builder.Build();

  // Create and run the simulator.
  drake::systems::Simulator<double> simulator(*diagram,
                                              diagram->CreateDefaultContext());
  simulator.set_target_realtime_rate(sim_parameters.target_realtime_rate);
  simulator.AdvanceTo(sim_parameters.simulation_time);

  return 0;
}

}  // namespace
}  // namespace hsr
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  return drake::examples::hsr::DoMain();
}
