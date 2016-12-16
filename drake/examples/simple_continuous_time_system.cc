
// Simple Continuous Time System Example
//
// This is meant to be a sort of "hello world" example for the
// drake::system classes.  It defines a very simple continuous time system,
// simulates it from a given initial condition, and plots the result.

#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/leaf_system.h"

// Simple Continuous Time System
//   xdot = -x + x^3
//   y = x
class SimpleContinuousTimeSystem : public drake::systems::LeafSystem<double> {
 public:
  SimpleContinuousTimeSystem() {
    const int kSize = 1;  // The dimension of both output (y) and state (x).
    this->DeclareOutputPort(drake::systems::kVectorValued, kSize);
    this->DeclareContinuousState(kSize);
  }

  // xdot = -x + x^3
  void DoCalcTimeDerivatives(
      const drake::systems::Context<double>& context,
      drake::systems::ContinuousState<double>* derivatives) const override {
    double x = context.get_continuous_state_vector().GetAtIndex(0);
    double xdot = -x + std::pow(x, 3.0);
    derivatives->get_mutable_vector()->SetAtIndex(0, xdot);
  }

  // y = x
  void DoCalcOutput(
      const drake::systems::Context<double>& context,
      drake::systems::SystemOutput<double>* output) const override {
    double x = context.get_continuous_state_vector().GetAtIndex(0);
    output->GetMutableVectorData(0)->SetAtIndex(0, x);
  }
};


int main(int argc, char* argv[]) {
  // Create the simple system.
  SimpleContinuousTimeSystem system;

  // Create the simulator.
  drake::systems::Simulator<double> simulator(system);

  // Set the initial conditions x(0).
  drake::systems::ContinuousState<double>& xc =
      *simulator.get_mutable_context()->get_mutable_continuous_state();
  xc[0] = 0.9;

  // Simulate for 10 seconds.
  simulator.StepTo(10);

  // Make sure the simulation converges to the stable fixed point at x=0.
  DRAKE_DEMAND(xc[0] < 1.0e-4);

  // TODO(russt): make a plot of the resulting trajectory.

  return 0;
}
