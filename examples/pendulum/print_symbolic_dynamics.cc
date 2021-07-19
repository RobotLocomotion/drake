#include <iostream>

#include "drake/common/find_resource.h"
#include "drake/common/symbolic.h"
#include "drake/examples/pendulum/pendulum_plant.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/multibody_plant.h"

// A simple example of extracting the symbolic dynamics of the pendulum system,
// and printing them to std::out.

using drake::symbolic::Expression;
using drake::symbolic::Variable;
using drake::multibody::MultibodyPlant;
using drake::multibody::Parser;
using drake::systems::System;

namespace drake {
namespace examples {
namespace pendulum {
namespace {

// Obtains the dynamics using PendulumPlant (a System that directly models the
// dynamics).
VectorX<Expression> PendulumPlantDynamics() {
  // Load the Pendulum.urdf into a symbolic PendulumPlant.
  PendulumPlant<Expression> symbolic_plant;

  // Obtain the symbolic dynamics.
  auto context = symbolic_plant.CreateDefaultContext();
  symbolic_plant.get_input_port().FixValue(context.get(),
                                           Expression(Variable("tau")));
  context->get_mutable_continuous_state_vector().SetAtIndex(
      0, Variable("theta"));
  context->get_mutable_continuous_state_vector().SetAtIndex(
      1, Variable("thetadot"));
  const auto& derivatives = symbolic_plant.EvalTimeDerivatives(*context);
  return derivatives.CopyToVector();
}

// Obtains the dynamics using MultibodyPlant configured to model a pendulum.
VectorX<Expression> MultibodyPlantDynamics() {
  // Load the Pendulum.urdf into a symbolic MultibodyPlant.
  const char* const urdf_path = "drake/examples/pendulum/Pendulum.urdf";
  MultibodyPlant<double> plant(0.0);
  Parser parser(&plant);
  parser.AddModelFromFile(FindResourceOrThrow(urdf_path));
  plant.Finalize();
  auto symbolic_plant_ptr = System<double>::ToSymbolic(plant);
  const MultibodyPlant<Expression>& symbolic_plant = *symbolic_plant_ptr;

  // Obtain the symbolic dynamics.
  auto context = symbolic_plant.CreateDefaultContext();
  symbolic_plant.get_actuation_input_port().FixValue(
      context.get(), Expression(Variable("tau")));
  symbolic_plant.SetPositionsAndVelocities(
      context.get(), Vector2<Expression>(
          Variable("theta"), Variable("thetadot")));
  const auto& derivatives = symbolic_plant.EvalTimeDerivatives(*context);
  return derivatives.CopyToVector();
}

int main() {
  std::cout << "PendulumPlantDynamics:\n";
  auto dynamics = PendulumPlantDynamics();
  std::cout << "d/dt theta    = " << dynamics[0] << "\n";
  std::cout << "d/dt thetadot = " << dynamics[1] << "\n";
  std::cout << "\n";

  std::cout << "MultibodyPlantDynamics:\n";
  dynamics = MultibodyPlantDynamics();
  std::cout << "d/dt theta    = " << dynamics[0] << "\n";
  std::cout << "d/dt thetadot = " << dynamics[1] << "\n";

  return 0;
}

}  // namespace
}  // namespace pendulum
}  // namespace examples
}  // namespace drake

int main() {
  return drake::examples::pendulum::main();
}
