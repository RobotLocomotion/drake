// Simple Continuous Time System Example
//
// This is meant to be a sort of "hello world" example for the
// drake::system classes.  It defines a very simple continuous time system,
// simulates it from a given initial condition, and checks the result.

#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace systems {
namespace {

// Simple Continuous Time System
//   xdot = -x + x³
//   y = x
class SimpleContinuousTimeSystem : public LeafSystem<double> {
 public:
  SimpleContinuousTimeSystem() {
    auto state_index = DeclareContinuousState(1);  // One state variable.
    DeclareStateOutputPort("y", state_index);
  }

 private:
  // xdot = -x + x³
  void DoCalcTimeDerivatives(
      const Context<double>& context,
      ContinuousState<double>* derivatives) const override {
    const double x = context.get_continuous_state()[0];
    const double xdot = -x + std::pow(x, 3.0);
    (*derivatives)[0] = xdot;
  }
};

int main() {
  // Create the simple system.
  SimpleContinuousTimeSystem system;

  // Create the simulator.
  Simulator<double> simulator(system);

  // Set the initial conditions x(0).
  ContinuousState<double>& state =
      simulator.get_mutable_context().get_mutable_continuous_state();
  state[0] = 0.9;

  // Simulate for 10 seconds.
  simulator.AdvanceTo(10);

  // Make sure the simulation converges to the stable fixed point at x=0.
  DRAKE_DEMAND(state[0] < 1.0e-4);

  // TODO(russt): make a plot of the resulting trajectory.

  return 0;
}

}  // namespace
}  // namespace systems
}  // namespace drake

int main() {
  return drake::systems::main();
}
