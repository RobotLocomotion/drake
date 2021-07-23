#include <fstream>
#include <iostream>

#include <gflags/gflags.h>
#include <yaml-cpp/yaml.h>

#include "drake/common/name_value.h"
#include "drake/common/schema/stochastic.h"
#include "drake/common/yaml/yaml_read_archive.h"
#include "drake/common/yaml/yaml_write_archive.h"
#include "drake/examples/acrobot/acrobot_plant.h"
#include "drake/examples/acrobot/spong_controller.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/vector_log_sink.h"

using drake::examples::acrobot::AcrobotParams;
using drake::examples::acrobot::AcrobotPlant;
using drake::examples::acrobot::AcrobotSpongController;
using drake::systems::DiagramBuilder;
using drake::systems::Simulator;

namespace drake {
namespace examples {
namespace acrobot {
namespace {

DEFINE_string(scenario, "", "Scenario file to load (required).");
DEFINE_string(dump_scenario, "", "Scenario file to save.");
DEFINE_string(output, "", "Output file to save (required).");
DEFINE_int32(random_seed, drake::RandomGenerator::default_seed,
             "Random seed");

// The YAML format for --scenario.
struct Scenario {
  // TODO(jeremy.nimmer) Maybe use AcrobotParams class for this one?
  drake::schema::DistributionVectorVariant<4> controller_params =
      Eigen::Vector4d::Constant(NAN);
  drake::schema::DistributionVectorVariant<4> initial_state =
      Eigen::Vector4d::Constant(NAN);
  double t_final = NAN;
  double tape_period = NAN;

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(controller_params));
    a->Visit(DRAKE_NVP(initial_state));
    a->Visit(DRAKE_NVP(t_final));
    a->Visit(DRAKE_NVP(tape_period));
  }
};

// The YAML format for --output.
struct Output {
  Eigen::MatrixXd x_tape;

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(x_tape));
  }
};

// If the Scenario `input` has any randomness, sample it; return a Scenario
// with no randomness (ie, IsDeterministic() is guaranteed true on all
// elements).
Scenario SampleScenario(const Scenario& input) {
  drake::RandomGenerator random(FLAGS_random_seed);
  Scenario result = input;
  result.controller_params =
      drake::schema::ToDistributionVector(input.controller_params)->
      Sample(&random);
  result.initial_state =
      drake::schema::ToDistributionVector(input.initial_state)->
      Sample(&random);
  return result;
}

// Simulates an Acrobot + Spong controller from the given initial state and
// parameters until the given final time.  Returns a YAML string of the state.
std::string Simulate(const YAML::Node& scenario_node) {
  Scenario scenario;
  drake::yaml::YamlReadArchive(scenario_node).Accept(&scenario);

  // Resolve scenario randomness and write out the resolved scenario.
  scenario = SampleScenario(scenario);
  if (!FLAGS_dump_scenario.empty()) {
    std::ofstream out(FLAGS_dump_scenario);
    DRAKE_THROW_UNLESS(out.good());
    drake::yaml::YamlWriteArchive writer;
    writer.Accept(scenario);
    out << writer.EmitString();
  }

  DiagramBuilder<double> builder;
  auto plant = builder.AddSystem<AcrobotPlant>();
  auto controller = builder.AddSystem<AcrobotSpongController>();

  builder.Connect(plant->get_output_port(0), controller->get_input_port(0));
  builder.Connect(controller->get_output_port(0), plant->get_input_port(0));
  auto state_logger = LogVectorOutput(plant->get_output_port(0), &builder,
                                      scenario.tape_period);

  auto diagram = builder.Build();
  Simulator<double> simulator(*diagram);
  auto& context = simulator.get_mutable_context();

  auto& plant_context = diagram->GetMutableSubsystemContext(
      *plant, &context);
  DRAKE_DEMAND(drake::schema::IsDeterministic(scenario.initial_state));
  plant_context.SetContinuousState(
      drake::schema::GetDeterministicValue(scenario.initial_state));

  auto& controller_context = diagram->GetMutableSubsystemContext(
      *controller, &context);
  DRAKE_DEMAND(drake::schema::IsDeterministic(
      scenario.controller_params));
  controller_context.get_mutable_numeric_parameter(0).SetFromVector(
      drake::schema::GetDeterministicValue(scenario.controller_params));

  simulator.AdvanceTo(scenario.t_final);

  Output output;
  output.x_tape = state_logger->FindLog(context).data();
  drake::yaml::YamlWriteArchive writer;
  writer.Accept(output);
  // The EmitString call below saves a document like so:
  //
  // doc:
  //   x_tape: [[0,1,2,3,4,5],[0,1,2,3,4,5],[0,1,2,3,4,5],[0,1,2,3,4,5]]
  //
  // When writing to disk, we need to strip off the "doc:\n  " part in order to
  // meet the expectations of our caller.
  // TODO(jeremy.nimmer) Repair the YamlWriteArchive APIs so we don't have to
  // strip off this prefix.
  const std::string result = writer.EmitString("doc").substr(7);
  DRAKE_DEMAND(result.substr(0, 7) == "x_tape:");
  return result;
}

int Main() {
  if (FLAGS_scenario.empty()) {
    std::cerr << "A --scenario file is required.\n";
    return 1;
  }
  if (FLAGS_output.empty()) {
    std::cerr << "An --output file is required.\n";
    return 1;
  }
  std::ifstream in(FLAGS_scenario);
  if (!in.good()) {
    std::cerr << "Could not read from '" << FLAGS_scenario << "'.\n";
    return 1;
  }
  YAML::Node scenario_nodes = YAML::Load(in);
  if (!(scenario_nodes.IsMap() && scenario_nodes.size() == 1)) {
    std::cerr << "Scenario file does not have exactly 1 scenario\n";
    return 1;
  }
  const std::string name = scenario_nodes.begin()->first.Scalar();
  std::ofstream out(FLAGS_output);
  if (!out.good()) {
    std::cerr << "Could not write to '" << FLAGS_output << "'.\n";
    return 1;
  }
  const std::string output = Simulate(scenario_nodes[name]);
  out << output;
  return 0;
}

}  // namespace
}  // namespace acrobot
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::SetUsageMessage(
      "A main() program simulates a spong-controlled acrobot.");
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return drake::examples::acrobot::Main();
}
