/// @file
///
/// Runs the simulator for the valkyrie robot. It receives torque commands and
/// sends measured robot state through LCM traffic. See valkyrie_simulator.h
/// for more details.

#include "drake/examples/valkyrie/valkyrie_constants.h"
#include "drake/examples/valkyrie/valkyrie_simulator.h"
#include "drake/systems/analysis/semi_explicit_euler_integrator.h"
#include "drake/systems/analysis/simulator.h"

namespace drake {
namespace examples {
namespace valkyrie {

int main() {
  // LCM communication.
  lcm::DrakeLcm lcm;
  ValkyrieSimulationDiagram diagram(&lcm);

  // Create simulator.
  systems::Simulator<double> simulator(diagram);
  systems::Context<double>& context = simulator.get_mutable_context();

  // Integrator set arbitrarily. The time step was selected by tuning for the
  // largest value that appears to give stable results.
  simulator.reset_integrator<systems::SemiExplicitEulerIntegrator<double>>(
      diagram, 3e-4, &context);
  simulator.set_publish_every_time_step(false);

  // Set initial state.
  auto plant = diagram.get_mutable_plant();
  systems::Context<double>& plant_context =
      diagram.GetMutableSubsystemContext(*plant, &context);

  // TODO(tkoolen): make it easy to specify a different initial configuration.
  VectorX<double> initial_state = RPYValkyrieFixedPointState();
  plant->set_state_vector(&plant_context, initial_state);
  lcm.StartReceiveThread();

  simulator.StepTo(std::numeric_limits<double>::infinity());
  return 0;
}

}  // namespace valkyrie
}  // namespace examples
}  // namespace drake

int main() {
  return drake::examples::valkyrie::main();
}
