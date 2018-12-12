#include <memory>
#include <string>
#include <vector>

#include <gflags/gflags.h>
#include "optitrack/optitrack_frame_t.hpp"

#include "drake/common/text_logging_gflags.h"
#include "drake/examples/kuka_iiwa_arm/dev/pick_and_place/lcm_plant.h"
#include "drake/examples/kuka_iiwa_arm/iiwa_lcm.h"
#include "drake/examples/kuka_iiwa_arm/pick_and_place/pick_and_place_configuration.h"
#include "drake/examples/kuka_iiwa_arm/pick_and_place/pick_and_place_configuration_parsing.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/lcmt_contact_results_for_viz.hpp"
#include "drake/lcmt_iiwa_command.hpp"
#include "drake/lcmt_iiwa_status.hpp"
#include "drake/lcmt_schunk_wsg_command.hpp"
#include "drake/lcmt_schunk_wsg_status.hpp"
#include "drake/manipulation/schunk_wsg/schunk_wsg_lcm.h"
#include "drake/multibody/rigid_body_plant/contact_results_to_lcm.h"
#include "drake/multibody/rigid_body_plant/drake_visualizer.h"
#include "drake/systems/analysis/runge_kutta2_integrator.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"

DEFINE_double(simulation_sec, std::numeric_limits<double>::infinity(),
              "Number of seconds to simulate.");
DEFINE_double(dt, 5e-4, "Integration step size");
DEFINE_double(realtime_rate, 1.0,
              "Rate at which to run the simulation, "
              "relative to realtime");
DEFINE_bool(quick, false,
            "Run only a brief simulation and return success "
            "without executing the entire task");
DEFINE_string(configuration_file,
              "drake/examples/kuka_iiwa_arm/pick_and_place/configuration/"
              "yellow_posts.pick_and_place_configuration",
              "Path to the configuration file.");

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {
namespace pick_and_place {
namespace {

int DoMain(void) {
  // Parse the configuration file.
  const pick_and_place::SimulatedPlantConfiguration plant_configuration =
      pick_and_place::ParseSimulatedPlantConfigurationOrThrow(
          FLAGS_configuration_file);
  const pick_and_place::OptitrackConfiguration optitrack_configuration =
      pick_and_place::ParseOptitrackConfigurationOrThrow(
          FLAGS_configuration_file);

  // Instantiate an LCM instance for use with publishers and subscribers.
  lcm::DrakeLcm lcm;

  // Begin setting up the diagram.
  systems::DiagramBuilder<double> builder;

  // Add the plant.
  auto plant = builder.AddSystem<pick_and_place::LcmPlant>(
      plant_configuration, optitrack_configuration);

  // Add the visualizer.
  auto drake_visualizer =
      builder.AddSystem<systems::DrakeVisualizer>(plant->get_tree(), &lcm);
  drake_visualizer->set_publish_period(kIiwaLcmStatusPeriod);
  builder.Connect(plant->get_output_port_plant_state(),
                  drake_visualizer->get_input_port(0));

  // Publish the contact results over LCM.
  auto contact_viz =
      builder.AddSystem<systems::ContactResultsToLcmSystem<double>>(
          plant->get_tree());
  auto contact_results_publisher = builder.AddSystem(
      systems::lcm::LcmPublisherSystem::Make<lcmt_contact_results_for_viz>(
          "CONTACT_RESULTS", &lcm));
  builder.Connect(plant->get_output_port_contact_results(),
                  contact_viz->get_input_port(0));
  builder.Connect(contact_viz->get_output_port(0),
                  contact_results_publisher->get_input_port());
  contact_results_publisher->set_publish_period(kIiwaLcmStatusPeriod);

  // Publish the mock Optitrack data over LCM.
  auto optitrack_pub = builder.AddSystem(
      systems::lcm::LcmPublisherSystem::Make<optitrack::optitrack_frame_t>(
          "OPTITRACK_FRAMES", &lcm));
  optitrack_pub->set_publish_period(kIiwaLcmStatusPeriod);
  builder.Connect(plant->get_output_port_optitrack_frame(),
                  optitrack_pub->get_input_port());

  // Add LCM subscribers/publishers for the iiwa arms.
  const int num_iiwa{plant->num_iiwa()};
  kuka_iiwa_arm::IiwaCommandTranslator iiwa_cmd_to_vec;
  for (int i = 0; i < num_iiwa; ++i) {
    // All LCM traffic for this arm will occur on channels that end with the
    // suffix specified below.
    const std::string suffix = (num_iiwa > 1) ? "_" + std::to_string(i) : "";
    auto iiwa_command_sub =
        builder.AddSystem(std::make_unique<systems::lcm::LcmSubscriberSystem>(
            "IIWA_COMMAND" + suffix, iiwa_cmd_to_vec, &lcm));

    iiwa_command_sub->set_name("iiwa_command_subscriber" + suffix);
    builder.Connect(iiwa_command_sub->get_output_port(),
                    plant->get_input_port_iiwa_command(i));

    auto iiwa_status_pub = builder.AddSystem(
        systems::lcm::LcmPublisherSystem::Make<lcmt_iiwa_status>(
            "IIWA_STATUS" + suffix, &lcm));
    iiwa_status_pub->set_name("iiwa_status_publisher" + suffix);
    iiwa_status_pub->set_publish_period(kIiwaLcmStatusPeriod);
    builder.Connect(plant->get_output_port_iiwa_status(i),
                    iiwa_status_pub->get_input_port());
  }

  // Add LCM subscribers/publishers for the Schunk grippers.
  manipulation::schunk_wsg::SchunkWsgCommandTranslator wsg_cmd_to_vec;
  const int num_wsg{plant->num_wsg()};
  for (int i = 0; i < num_wsg; ++i) {
    // All LCM traffic for this gripper will occur on channels that end with the
    // suffix specified below.
    const std::string suffix = (num_wsg > 1) ? "_" + std::to_string(i) : "";
    auto wsg_status_pub = builder.AddSystem(
        systems::lcm::LcmPublisherSystem::Make<lcmt_schunk_wsg_status>(
            "SCHUNK_WSG_STATUS" + suffix, &lcm));
    builder.Connect(plant->get_output_port_wsg_status(i),
                    wsg_status_pub->get_input_port());
    wsg_status_pub->set_publish_period(kIiwaLcmStatusPeriod);

    auto wsg_command_sub =
        builder.AddSystem(std::make_unique<systems::lcm::LcmSubscriberSystem>(
            "SCHUNK_WSG_COMMAND" + suffix, wsg_cmd_to_vec, &lcm));
    builder.Connect(wsg_command_sub->get_output_port(),
                    plant->get_input_port_wsg_command(i));
  }

  // Build the diagram.
  auto sys = builder.Build();

  // Create and configure the simulator.
  systems::Simulator<double> simulator(*sys);
  simulator.set_target_realtime_rate(FLAGS_realtime_rate);
  simulator.reset_integrator<systems::RungeKutta2Integrator<double>>(
      *sys, FLAGS_dt, &simulator.get_mutable_context());
  simulator.get_mutable_integrator()->set_maximum_step_size(FLAGS_dt);
  simulator.get_mutable_integrator()->set_fixed_step_mode(true);
  simulator.set_publish_every_time_step(false);
  simulator.Initialize();

  // Simulate for the specified duration.
  lcm.StartReceiveThread();
  simulator.StepTo(FLAGS_simulation_sec);

  lcm.StopReceiveThread();
  return 0;
}

}  // namespace
}  // namespace pick_and_place
}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  drake::logging::HandleSpdlogGflags();
  return drake::examples::kuka_iiwa_arm::pick_and_place::DoMain();
}
