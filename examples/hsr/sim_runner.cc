#include <memory>

#include <gflags/gflags.h>

#include "drake/examples/hsr/hsr_world.h"
#include "drake/examples/hsr/parameters/sim_parameters.h"
#include "drake/examples/hsr/sim_command_receiver.h"
#include "drake/examples/hsr/sim_status_sender.h"
#include "drake/geometry/geometry_visualization.h"
#include "drake/multibody/plant/contact_results_to_lcm.h"
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
  // Build a generic multibody plant.
  systems::DiagramBuilder<double> builder;
  auto hsr_world = builder.AddSystem<HsrWorld<double>>("");

  geometry::ConnectDrakeVisualizer(&builder,
                                   hsr_world->get_mutable_scene_graph(),
                                   hsr_world->GetOutputPort("pose_bundle"));

  multibody::ConnectContactResultsToDrakeVisualizer(
      &builder, hsr_world->get_mutable_multibody_plant(),
      hsr_world->GetOutputPort("contact_results"));

  const auto& sim_parameters = hsr::parameters::hsr_sim_flags();
  const auto& hsr_plant = hsr_world->get_hsr_plant("hsr");
  if (sim_parameters.use_constant_desired_state) {
    // Add a constant source to set desired position and velocity. These values
    // are set arbitrarily.
    drake::VectorX<double> constant_pos_value = drake::VectorX<double>::Zero(
        hsr_plant.num_positions() + hsr_plant.num_velocities());
    // Set the w of the quaternion base to be 1.0 to make the state to be valid.
    constant_pos_value(0) = 0.5;
    const std::string joint_name = "head_tilt_joint";
    constant_pos_value[hsr_plant.GetJointByName(joint_name).position_start()] =
        1;

    auto desired_pos_constant_source =
        builder.template AddSystem<systems::ConstantVectorSource<double>>(
            constant_pos_value);
    desired_pos_constant_source->set_name("desired_pos_constant_source");
    builder.Connect(desired_pos_constant_source->get_output_port(),
                    hsr_world->GetInputPort("hsr_desired_state"));
  } else {
    auto lcm = builder.AddSystem<systems::lcm::LcmInterfaceSystem>();

    // HSR command subscriber.
    auto hsr_command_subscriber = builder.AddSystem(
        systems::lcm::LcmSubscriberSystem::Make<drake::lcmt_hsr_sim_command>(
            "hsr_sim_command_channel", lcm));
    auto hsr_command_receiver =
        builder.AddSystem<hsr::SimCommandReceiver>(&hsr_plant);
    builder.Connect(hsr_command_subscriber->get_output_port(),
                    hsr_command_receiver->get_sim_command_input_port());

    builder.Connect(hsr_command_receiver->get_desired_state_output_port(),
                    hsr_world->GetInputPort("hsr_desired_state"));

    // HSR status publisher.
    auto hsr_status_publisher = builder.AddSystem(
        systems::lcm::LcmPublisherSystem::Make<drake::lcmt_hsr_sim_status>(
            "hsr_sim_status_channel", lcm, 0.005 /* publish period */));
    auto hsr_status_sender =
        builder.AddSystem<hsr::SimStatusSender>(&hsr_plant);
    builder.Connect(hsr_status_sender->get_sim_status_output_port(),
                    hsr_status_publisher->get_input_port());
    builder.Connect(hsr_world->GetOutputPort("hsr_state_estimated"),
                    hsr_status_sender->get_estimated_state_input_port());
  }

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
