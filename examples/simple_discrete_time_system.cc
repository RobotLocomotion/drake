// Simple Discrete Time System Example
//
// This is meant to be a sort of "hello world" example for the
// drake::system classes.  It defines a very simple discrete time system,
// simulates it from a given initial condition, and checks the result.

#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/leaf_system.h"

// Simple Discrete Time System
//   x_{n+1} = x_n³
//         y = x
class SimpleDiscreteTimeSystem : public drake::systems::LeafSystem<double> {
 public:
  SimpleDiscreteTimeSystem() {
    this->DeclarePeriodicDiscreteUpdateEvent(1.0, 0.0,
                                             &SimpleDiscreteTimeSystem::Update);
    this->DeclareVectorOutputPort("y", drake::systems::BasicVector<double>(1),
                                  &SimpleDiscreteTimeSystem::CopyStateOut);
    this->DeclareDiscreteState(1);  // One state variable.
  }

 private:
  // x_{n+1} = x_n³
  drake::systems::EventStatus Update(
      const drake::systems::Context<double>& context,
      drake::systems::DiscreteValues<double>* next_state) const {
    const double x_n = context.get_discrete_state()[0];
    (*next_state)[0] = std::pow(x_n, 3.0);
    return drake::systems::EventStatus::Succeeded();
  }

  // y = x
  void CopyStateOut(const drake::systems::Context<double>& context,
                    drake::systems::BasicVector<double>* output) const {
    const double x = context.get_discrete_state()[0];
    (*output)[0] = x;
  }
};

int main() {
  // Create the simple system.
  SimpleDiscreteTimeSystem system;

  // Create the simulator.
  drake::systems::Simulator<double> simulator(system);

  // Set the initial conditions x₀.
  drake::systems::DiscreteValues<double>& state =
      simulator.get_mutable_context().get_mutable_discrete_state();
  state[0] = 0.99;

  // Simulate for 10 seconds.
  simulator.StepTo(10);

  // Make sure the simulation converges to the stable fixed point at x=0.
  DRAKE_DEMAND(state[0] < 1.0e-4);

  // TODO(russt): make a plot of the resulting trajectory.

  return 0;
}
