
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
    this->DeclareUpdatePeriodSec(1.0);
    this->DeclareOutputPort(drake::systems::kVectorValued, kSize,
                            drake::systems::kDiscreteSampling);
    this->DeclareDifferenceState(kSize);
  }

  // x[n+1] = x[n]^3
  void DoEvalDifferenceUpdates(
      const drake::systems::Context<double>& context,
      drake::systems::DifferenceState<double>* updates) const override {
    double x = context.get_difference_state(0)->GetAtIndex(0);
    double xn = std::pow(x, 3.0);
    updates->get_mutable_difference_state(0)->SetAtIndex(0, xn);
  }

  // y = x
  void EvalOutput(const drake::systems::Context<double>& context,
                  drake::systems::SystemOutput<double>* output) const override {
    output->GetMutableVectorData(0)->SetFromVector(
        context.get_difference_state(0)->CopyToVector());
  }
};

int main(int argc, char* argv[]) {
  // create the simple system
  SimpleDiscreteTimeSystem system;

  // create the simulator
  drake::systems::Simulator<double> simulator(system);

  // set the initial conditions x(0);
  drake::systems::DifferenceState<double>& xd =
      *simulator.get_mutable_context()->get_mutable_difference_state();
  xd.get_mutable_difference_state(0)->SetAtIndex(0, 0.99);

  // simulate for 10 seconds
  simulator.StepTo(10);

  // make sure the simulation converges to the stable fixed point at x=0
  DRAKE_DEMAND(xd.get_difference_state(0)->GetAtIndex(0) < 1.0e-4);

  // TODO(russt): make a plot of the resulting trajectory (using vtk?)

  return 0;
}
