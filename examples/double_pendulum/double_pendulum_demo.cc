///
/// @brief  An SDF based double pendulum example.
///

#include <limits>
#include <memory>

#include "drake/common/find_resource.h"
#include "drake/common/text_logging_gflags.h"
#include "drake/examples/double_pendulum/sdf_helpers.h"
#include "drake/multibody/rigid_body_plant/drake_visualizer.h"
#include "drake/multibody/rigid_body_plant/rigid_body_plant.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_interface_system.h"

DEFINE_double(realtime_rate, 1.0,
              "Rate at which to run the simulation, relative to realtime");
DEFINE_double(simulation_time, std::numeric_limits<double>::infinity(),
              "How long to simulate the pendulum");

namespace drake {
namespace examples {
namespace double_pendulum {
namespace {

// Fixed path to double pendulum SDF model.
static const char* const kDoublePendulumSdfPath =
  "drake/examples/double_pendulum/models/double_pendulum.sdf";

//
// Main function for demo.
//
int main(int argc, char* argv[]) {
  gflags::SetUsageMessage("Simple sdformat usage example, just"
                          "make sure drake-visualizer is running!");
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  logging::HandleSpdlogGflags();
  // Load and parse double pendulum SDF from file into a tree.
  const std::string sdf_path = FindResourceOrThrow(kDoublePendulumSdfPath);
  auto tree = std::make_unique<RigidBodyTree<double>>();
  ParseModelFromFile(sdf_path, tree.get());
  tree->compile();
  // Instantiate builder.
  systems::DiagramBuilder<double> builder;
  // Instantiate LCM interface.
  auto lcm = builder.AddSystem<systems::lcm::LcmInterfaceSystem>();
  // Move double pendulum tree to plant.
  auto plant = builder.template AddSystem<systems::RigidBodyPlant<double>>(
      std::move(tree));
  // Add visualizer client.
  auto visualizer = builder.template AddSystem<systems::DrakeVisualizer>(
      plant->get_rigid_body_tree(), lcm);
  // Wire all blocks together.
  builder.Connect(plant->get_output_port(0),
                  visualizer->get_input_port(0));
  auto diagram = builder.Build();
  // Instantiate and configure simulator.
  auto simulator = std::make_unique<systems::Simulator<double>>(*diagram);
  simulator->set_target_realtime_rate(FLAGS_realtime_rate);
  simulator->Initialize();
  // Run simulation.
  simulator->AdvanceTo(FLAGS_simulation_time);
  return 0;
}

}  // namespace
}  // namespace double_pendulum
}  // namespace examples
}  // namespace drake

int main(int argc, char **argv) {
  return drake::examples::double_pendulum::main(argc, argv);
}
