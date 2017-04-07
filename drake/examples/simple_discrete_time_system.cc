
// Simple Discrete Time System Example
//
// This is meant to be a sort of "hello world" example for the
// drake::system classes.  It defines a very simple discrete time system,
// simulates it from a given initial condition, and plots the result.

#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/leaf_system.h"

// Simple Discrete Time System
//   x[n+1] = x[n]^3
//   y = x
class SimpleDiscreteTimeSystem : public drake::systems::LeafSystem<double> {
 public:
  SimpleDiscreteTimeSystem() {
    const int kSize = 1;  // The dimension of both output (y) and state (x).
    this->DeclareDiscreteUpdatePeriodSec(1.0);
    this->DeclareVectorOutputPort(drake::systems::BasicVector<double>(kSize),
                                  &SimpleDiscreteTimeSystem::CopyStateOut);
    this->DeclareDiscreteState(kSize);
  }

 private:
  // x[n+1] = x[n]^3
  void DoCalcDiscreteVariableUpdates(
      const drake::systems::Context<double>& context,
      drake::systems::DiscreteValues<double>* updates) const override {
    double x = context.get_discrete_state(0)->GetAtIndex(0);
    double xn = std::pow(x, 3.0);
    (*updates)[0] = xn;
  }

  // y = x
  void CopyStateOut(const drake::systems::Context<double>& context,
                    drake::systems::BasicVector<double>* output) const {
    output->SetFromVector(
        context.get_discrete_state(0)->CopyToVector());
  }
};

int main() {
  // Create the simple system.
  SimpleDiscreteTimeSystem system;

  // Create the simulator.
  drake::systems::Simulator<double> simulator(system);

  // Set the initial conditions x(0).
  drake::systems::DiscreteValues<double>& xd =
      *simulator.get_mutable_context()->get_mutable_discrete_state();
  xd[0] = 0.99;

  // Simulate for 10 seconds.
  simulator.StepTo(10);

  // Make sure the simulation converges to the stable fixed point at x=0.
  DRAKE_DEMAND(xd[0] < 1.0e-4);

  // TODO(russt): make a plot of the resulting trajectory.

  return 0;
}
