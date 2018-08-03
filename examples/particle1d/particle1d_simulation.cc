#include <memory>

#include <gflags/gflags.h>

#include "drake/common/find_resource.h"
#include "drake/examples/particle1d/particle1d_plant.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_plant/drake_visualizer.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/constant_vector_source.h"

namespace drake {
namespace examples {
namespace particle1d {
namespace {

int DoMain() {
  // Parsing the URDF and constructing a RigidBodyTree from it
  auto tree =
      std::make_unique<RigidBodyTree<double>>();
  parsers::urdf::AddModelInstanceFromUrdfFileToWorld(
      FindResourceOrThrow("drake/examples/particle1d/particle1d.urdf"),
      multibody::joints::kFixed, tree.get());

  // Construct and empty diagram, then add a particle plant.
  systems::DiagramBuilder<double> builder;

  Particle1dPlant<double>* particle_plant =
      builder.AddSystem<Particle1dPlant<double>>();
  particle_plant->set_name("RigidBox");

  // To set constant parameters (such as mass) in the particle plant, pass
  // information from the tree (which read constant parameters in the .urdf).
  particle_plant->SetConstantParameters(*tree);

  // Add a visualizer block to the diagram, uses the tree and an lcm object.
  lcm::DrakeLcm lcm;
  systems::DrakeVisualizer* publisher =
      builder.AddSystem<systems::DrakeVisualizer>(*tree, &lcm);
  publisher->set_name("publisher");

  // Within the diagram, connect the particle plant's output port to the
  // visualizer's input port.
  builder.Connect(particle_plant->get_output_port(),
                  publisher->get_input_port(0));

  // Build the diagram described by previous call to connect input/export port.
  auto diagram = builder.Build();

  // Constructs a Simulator that advances this diagram through time.
  systems::Simulator<double> simulator(*diagram);

  // To set initial values for the simulation:
  // * Get the Simulator's context.
  // * Get the part of the Simulator's context associated with particle_plant.
  // * Get the part of the particle_plant's context associated with state.
  // * Fill the state with initial values.
  systems::Context<double>& simulator_context = simulator.get_mutable_context();
  systems::Context<double>& particle_plant_context =
      diagram->GetMutableSubsystemContext(*particle_plant, &simulator_context);
  systems::BasicVector<double>& state =
      particle_plant->get_mutable_state(&particle_plant_context);

  const double x_init = 0.0;
  const double xDt_init = 0.0;
  state.SetAtIndex(0, x_init);
  state.SetAtIndex(1, xDt_init);

  // Set upper-limit on simulation speed (mostly for visualization).
  simulator.set_target_realtime_rate(10);

  // Simulate for 10 seconds (default units are SI, with units of seconds).
  std::cout << "Starting simulation." << std::endl;
  simulator.StepTo(10.0);
  std::cout << "Simulation done." << std::endl;

  return 0;
}

} // namespace
} // namespace particle1d
} // namespace examples
} // drake

int main(int, char* []) {
  return drake::examples::particle1d::DoMain();
}
