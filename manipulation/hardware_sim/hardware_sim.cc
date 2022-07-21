/* This program serves as an example of a simulator for hardware, i.e., the
robots that one might have in their lab.

It is intended to operate in the "no ground truth" regime, i.e, the only LCM
messages it knows about are the ones used by the actual hardware.

There is no controller built-in to this program.  It merely sends status
and sensor messages, and listens for command messages.

The one novelty we entertain here that is different from real life is that we
emit drake-visualizer messages, so that you can watch a simulation on your
screen while some (separate) controller operates the robot, without extra
hassle. */

#include <cctype>
#include <chrono>

#include <gflags/gflags.h>

#include "drake/geometry/drake_visualizer.h"
#include "drake/manipulation/hardware_sim/scenario.h"
#include "drake/multibody/parsing/process_model_directives.h"
#include "drake/multibody/plant/multibody_plant_config_functions.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/analysis/simulator_config_functions.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_config_functions.h"

DEFINE_string(scenario_file, "", "Scenario filename");
DEFINE_string(scenario_name, "", "Scenario name within the file");
DEFINE_string(scenario_text, "{}", "Additional YAML scenario text to load");

namespace drake {
namespace {

using geometry::DrakeVisualizer;
using geometry::DrakeVisualizerParams;
using lcm::DrakeLcmInterface;
using multibody::ModelInstanceIndex;
using multibody::parsing::ProcessModelDirectives;
using systems::ApplySimulatorConfig;
using systems::Diagram;
using systems::DiagramBuilder;
using systems::Simulator;
using systems::lcm::ApplyLcmBusConfig;
using systems::lcm::LcmBuses;

// Class to hold the configuration and data of a simulation.
class Simulation {
 public:
  explicit Simulation(const Scenario& scenario)
      : scenario_(scenario) {}

  // Performs all of the initial setup of the simulation diagram and context.
  void Setup();

  // Runs the main loop of the simulation until completion criteria are met.
  void Simulate();

 private:
  // The scenario passed into the ctor.
  const Scenario scenario_;

  std::unique_ptr<Diagram<double>> diagram_;
  std::unique_ptr<Simulator<double>> simulator_;
};

void Simulation::Setup() {
  DiagramBuilder<double> builder;

  // Create the multibody plant and scene graph.
  auto [sim_plant, scene_graph] =
      AddMultibodyPlant(scenario_.plant_config, &builder);

  // Add model directives.
  ProcessModelDirectives({scenario_.directives}, &sim_plant);

  // Add LCM buses. (The simulator will handle polling the network for new
  // messages and dispatching them to the receivers, i.e., "pump" the bus.)
  const LcmBuses lcm_buses = ApplyLcmBusConfig(scenario_.lcm_buses, &builder);

  // Now the plant is complete.
  sim_plant.Finalize();

  // Add visualizer(s).
  // TODO(jwnimmer-tri) Sugar for this.
  DrakeLcmInterface* lcm = lcm_buses.Find("hardware_sim visualizer", "default");
  DrakeVisualizer<double>::AddToBuilder(&builder, scene_graph, lcm);
#if 0
  DrakeVisualizerParams params{.role = geometry::Role::kProximity,
                               .default_color = {1.0, 0.0, 0.0, 0.5},
                               .use_role_channel_suffix = true};
  DrakeVisualizer<double>::AddToBuilder(&builder, scene_graph, lcm, params);
#endif

  // Build the diagram and its simulator.
  diagram_ = builder.Build();
  simulator_ = std::make_unique<Simulator<double>>(*diagram_);
  ApplySimulatorConfig(simulator_.get(), scenario_.simulator_config);

  // Sample the random elements of the context.
  RandomGenerator random(scenario_.random_seed);
  diagram_->SetRandomContext(&simulator_->get_mutable_context(), &random);

  // TODO(jwnimmer-tri) Until we add driver stacks to the scenario, we need to
  // placate MbP's requirement for actuation input.
  for (ModelInstanceIndex i{0}; i < sim_plant.num_model_instances(); ++i) {
    auto& input_port = sim_plant.get_actuation_input_port(i);
    const int size = input_port.size();
    auto& plant_context = sim_plant.GetMyMutableContextFromRoot(
        &simulator_->get_mutable_context());
    input_port.FixValue(&plant_context, Eigen::VectorXd::Zero(size));
  }
}

void Simulation::Simulate() {
  simulator_->AdvanceTo(scenario_.simulation_duration);
}

int main() {
  const Scenario scenario = LoadScenario(
      FLAGS_scenario_file, FLAGS_scenario_name, FLAGS_scenario_text);
  Simulation sim(scenario);
  sim.Setup();
  sim.Simulate();
  return 0;
}

}  // namespace
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return drake::main();
}
