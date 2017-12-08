#include <memory>
#include <string>
#include <vector>

#include <gflags/gflags.h>

#include "drake/common/find_resource.h"
#include "drake/common/text_logging_gflags.h"
#include "drake/examples/kuka_iiwa_arm/dev/monolithic_pick_and_place/monolithic_pick_and_place_system.h"
#include "drake/examples/kuka_iiwa_arm/dev/pick_and_place/pick_and_place_configuration_parsing.h"
#include "drake/examples/kuka_iiwa_arm/iiwa_lcm.h"
#include "drake/examples/kuka_iiwa_arm/pick_and_place/pick_and_place_configuration.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/lcmt_contact_results_for_viz.hpp"
#include "drake/multibody/rigid_body_plant/contact_results_to_lcm.h"
#include "drake/multibody/rigid_body_plant/drake_visualizer.h"
#include "drake/systems/analysis/runge_kutta2_integrator.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_publisher_system.h"

DEFINE_double(simulation_sec, std::numeric_limits<double>::infinity(),
              "Number of seconds to simulate.");
DEFINE_double(dt, 5e-4, "Integration step size");
DEFINE_double(realtime_rate, 1.0,
              "Rate at which to run the simulation, "
              "relative to realtime");
DEFINE_bool(quick, false,
            "Run only a brief simulation and return success "
            "without executing the entire task");
DEFINE_bool(single_move, false, "Move the object once and then stop.");
DEFINE_string(configuration_file,
              "drake/examples/kuka_iiwa_arm/dev/pick_and_place/configuration/"
              "yellow_posts.pick_and_place_configuration",
              "Path to the configuration file.");

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {
namespace monolithic_pick_and_place {
namespace {

using manipulation::planner::InterpolatorType;
using systems::RungeKutta2Integrator;
using systems::Simulator;

int DoMain(void) {
  // Parse configuration file
  const pick_and_place::SimulatedPlantConfiguration plant_configuration =
      pick_and_place::ParseSimulatedPlantConfigurationOrThrow(
          FLAGS_configuration_file);
  const pick_and_place::OptitrackConfiguration optitrack_configuration =
      pick_and_place::ParseOptitrackConfigurationOrThrow(
          FLAGS_configuration_file);
  const std::vector<pick_and_place::PlannerConfiguration>
      planner_configurations =
          pick_and_place::ParsePlannerConfigurationsOrThrow(
              FLAGS_configuration_file);

  lcm::DrakeLcm lcm;
  systems::DiagramBuilder<double> builder;

  auto plant = builder.AddSystem<MonolithicPickAndPlaceSystem>(
      plant_configuration, optitrack_configuration, planner_configurations,
      FLAGS_single_move);

  // Add blocks to publish contact results.
  auto contact_results_publisher = builder.AddSystem(
      systems::lcm::LcmPublisherSystem::Make<lcmt_contact_results_for_viz>(
          "CONTACT_RESULTS", &lcm));
  builder.Connect(plant->get_output_port_contact_results(),
                  contact_results_publisher->get_input_port(0));
  contact_results_publisher->set_publish_period(kIiwaLcmStatusPeriod);

  // Add visualizer.
  auto drake_visualizer =
      builder.AddSystem<systems::DrakeVisualizer>(plant->get_tree(), &lcm);
  drake_visualizer->set_publish_period(kIiwaLcmStatusPeriod);

  builder.Connect(plant->get_output_port_plant_state(),
                  drake_visualizer->get_input_port(0));

  auto sys = builder.Build();
  Simulator<double> simulator(*sys);
  simulator.set_target_realtime_rate(FLAGS_realtime_rate);
  simulator.reset_integrator<RungeKutta2Integrator<double>>(
      *sys, FLAGS_dt, &simulator.get_mutable_context());
  simulator.get_mutable_integrator()->set_maximum_step_size(FLAGS_dt);
  simulator.get_mutable_integrator()->set_fixed_step_mode(true);

  lcm.StartReceiveThread();
  simulator.set_publish_every_time_step(false);
  simulator.Initialize();

  // Initialize the plan interpolators
  plant->Initialize(&sys->GetMutableSubsystemContext(
      *plant, &simulator.get_mutable_context()));

  // Step the simulator in some small increment.  Between steps, check
  // to see if the state machine thinks we're done, and if so that the
  // object is near the target.
  const double simulation_step = 0.1;
  bool done{false};
  while (!done) {
    simulator.StepTo(simulator.get_context().get_time() + simulation_step);
    if (FLAGS_quick) {
      // We've run a single step, just get out now since we won't have
      // reached our destination.
      return 0;
    }
    done = plant->is_done(
        sys->GetSubsystemContext(*plant, simulator.get_context()));
  }
  return 0;
}

}  // namespace
}  // namespace monolithic_pick_and_place
}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  drake::logging::HandleSpdlogGflags();
  return drake::examples::kuka_iiwa_arm::monolithic_pick_and_place::DoMain();
}
