// This file is licensed under the MIT-0 License.
// See LICENSE-MIT-0.txt in the current directory.

/* This program serves as an example of a simulator for hardware, i.e., a
simulator for robots that one might have in their lab. There is no controller
built-in to this program -- it merely sends status and sensor messages, and
listens for command messages.

It is intended to operate in the "no ground truth" regime, i.e, the only LCM
messages it knows about are the ones used by the actual hardware. The one
messaging difference from real life is that we emit visualization messages (for
Meldis) so that you can watch a simulation on your screen while some (separate)
controller operates the robot, without extra hassle.

Drake maintainers should keep this file in sync with hardware_sim.py. */

#include <fstream>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include <gflags/gflags.h>

#include "drake/common/unused.h"
#include "drake/examples/hardware_sim/scenario.h"
#include "drake/manipulation/kuka_iiwa/iiwa_driver_functions.h"
#include "drake/manipulation/schunk_wsg/schunk_wsg_driver_functions.h"
#include "drake/manipulation/util/apply_driver_configs.h"
#include "drake/manipulation/util/named_positions_functions.h"
#include "drake/manipulation/util/zero_force_driver_functions.h"
#include "drake/multibody/parsing/process_model_directives.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/plant/multibody_plant_config_functions.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/analysis/simulator_config_functions.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_config_functions.h"
#include "drake/systems/sensors/camera_config_functions.h"
#include "drake/visualization/visualization_config_functions.h"

DEFINE_string(scenario_file, "",
              "Scenario filename, e.g., "
              "drake/examples/hardware_sim/example_scenarios.yaml");
DEFINE_string(
    scenario_name, "",
    "Scenario name within the scenario_file, e.g., Demo in the "
    "example_scenarios.yaml; scenario names appears as the keys of the "
    "YAML document's top-level mapping item");
DEFINE_string(
    scenario_text, "{}",
    "Additional YAML scenario text to load, in order to override values "
    "in the scenario_file, e.g., timeouts");
DEFINE_string(
    graphviz, "",
    "Dump the Simulator's Diagram to this file in Graphviz format as a "
    "debugging aid");

namespace drake {
namespace {

using internal::Scenario;
using lcm::DrakeLcmInterface;
using multibody::ModelInstanceIndex;
using multibody::parsing::ModelInstanceInfo;
using multibody::parsing::ProcessModelDirectives;
using systems::ApplySimulatorConfig;
using systems::Diagram;
using systems::DiagramBuilder;
using systems::Simulator;
using systems::lcm::ApplyLcmBusConfig;
using systems::lcm::LcmBuses;
using systems::sensors::ApplyCameraConfig;
using visualization::ApplyVisualizationConfig;

/* Class that holds the configuration and data of a simulation. */
class Simulation {
 public:
  explicit Simulation(const Scenario& scenario) : scenario_(scenario) {}

  /* Performs all of the initial setup of the simulation diagram and context. */
  void Setup();

  /* Runs the main loop of the simulation until completion criteria are met. */
  void Simulate();

  /* Provides read-only access to the diagram. */
  const Diagram<double>& diagram() { return *diagram_; }

 private:
  // The scenario passed into the ctor.
  const Scenario scenario_;

  std::unique_ptr<Diagram<double>> diagram_;
  std::unique_ptr<Simulator<double>> simulator_;
};

void Simulation::Setup() {
  DiagramBuilder<double> builder;

  // Create the multibody plant and scene graph.
  auto [sim_plant, scene_graph] = AddMultibodyPlant(
      scenario_.plant_config, scenario_.scene_graph_config, &builder);

  // Add model directives.
  std::vector<ModelInstanceInfo> added_models;
  ProcessModelDirectives({scenario_.directives}, &sim_plant, &added_models);

  // Override or supplement initial positions.
  manipulation::ApplyNamedPositionsAsDefaults(scenario_.initial_position,
                                              &sim_plant);

  // Now the plant is complete.
  sim_plant.Finalize();

  // Add LCM buses. (The simulator will handle polling the network for new
  // messages and dispatching them to the receivers, i.e., "pump" the bus.)
  const LcmBuses lcm_buses = ApplyLcmBusConfig(scenario_.lcm_buses, &builder);

  // Add actuation inputs.
  ApplyDriverConfigs(scenario_.model_drivers, sim_plant, added_models,
                     lcm_buses, &builder);

  // Add scene cameras.
  for (const auto& [yaml_name, camera] : scenario_.cameras) {
    unused(yaml_name);
    ApplyCameraConfig(camera, &builder, &lcm_buses);
  }

  // Add visualization.
  ApplyVisualizationConfig(scenario_.visualization, &builder, &lcm_buses);

  // Build the diagram and its simulator.
  diagram_ = builder.Build();
  simulator_ = std::make_unique<Simulator<double>>(*diagram_);
  ApplySimulatorConfig(scenario_.simulator_config, simulator_.get());

  // Sample the random elements of the context.
  RandomGenerator random(scenario_.random_seed);
  diagram_->SetRandomContext(&simulator_->get_mutable_context(), &random);
}

void Simulation::Simulate() {
  simulator_->AdvanceTo(scenario_.simulation_duration);
}

int main() {
  const Scenario scenario = internal::LoadScenario(
      FLAGS_scenario_file, FLAGS_scenario_name, FLAGS_scenario_text);
  Simulation sim(scenario);
  sim.Setup();
  if (!FLAGS_graphviz.empty()) {
    std::ofstream graphviz(FLAGS_graphviz);
    std::map<std::string, std::string> options;
    options.emplace("plant/split", "I/O");
    graphviz << sim.diagram().GetGraphvizString({}, options);
    DRAKE_THROW_UNLESS(graphviz.good());
  }
  sim.Simulate();
  return 0;
}

}  // namespace
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return drake::main();
}
