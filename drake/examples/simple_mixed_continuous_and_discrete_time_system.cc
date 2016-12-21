
// Simple Mixed Continuous-Time/Discrete-Time System Example
//
// This is meant to be a sort of "hello world" example for the
// drake::system classes.  It defines a very simple system with both continuous
// and discrete time dynamics, simulates it from a given initial condition, and
// plots the result.

#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/leaf_system.h"

// Simple Discrete Time System
//   x1[n+1] = x1[n]^3
//   x2dot = -x2 + x2^3
//   y = [x1;x2]
class SimpleMixedContinuousTimeDiscreteTimeSystem
    : public drake::systems::LeafSystem<double> {
 public:
  SimpleMixedContinuousTimeDiscreteTimeSystem() {
    const int kSize = 1;
    this->DeclareUpdatePeriodSec(1.0);
    this->DeclareOutputPort(drake::systems::kVectorValued, 2 * kSize);
    this->DeclareContinuousState(kSize);
    this->DeclareDiscreteState(kSize);
  }

  // x[n+1] = x[n]^3
  void DoCalcDiscreteVariableUpdates(
      const drake::systems::Context<double>& context,
      drake::systems::DiscreteState<double>* updates) const override {
    const double x = context.get_discrete_state(0)->GetAtIndex(0);
    const double xn = std::pow(x, 3.0);
    updates->get_mutable_discrete_state(0)->SetAtIndex(0, xn);
  }

  // xdot = -x + x^3
  void DoCalcTimeDerivatives(
      const drake::systems::Context<double>& context,
      drake::systems::ContinuousState<double>* derivatives) const override {
    const double x = context.get_continuous_state_vector().GetAtIndex(0);
    const double xdot = -x + std::pow(x, 3.0);
    derivatives->get_mutable_vector()->SetAtIndex(0, xdot);
  }

  // y = x
  void DoCalcOutput(
      const drake::systems::Context<double>& context,
      drake::systems::SystemOutput<double>* output) const override {
    const double x1 = context.get_discrete_state(0)->GetAtIndex(0);
    output->GetMutableVectorData(0)->SetAtIndex(0, x1);

    const double x2 = context.get_continuous_state_vector().GetAtIndex(0);
    output->GetMutableVectorData(0)->SetAtIndex(1, x2);
  }
};

int main(int argc, char* argv[]) {
  // Create the simple system.
  SimpleMixedContinuousTimeDiscreteTimeSystem system;

  // Create the simulator.
  drake::systems::Simulator<double> simulator(system);

  // Set the initial conditions x(0).
  drake::systems::DiscreteState<double>& xd =
      *simulator.get_mutable_context()->get_mutable_discrete_state();
  xd.get_mutable_discrete_state(0)->SetAtIndex(0, 0.99);
  drake::systems::ContinuousState<double>& xc =
      *simulator.get_mutable_context()->get_mutable_continuous_state();
  xc[0] = 0.9;

  // Simulate for 10 seconds.
  simulator.StepTo(10);

  // Make sure the simulation converges to the stable fixed point at x=0.
  DRAKE_DEMAND(xd.get_discrete_state(0)->GetAtIndex(0) < 1.0e-4);
  DRAKE_DEMAND(xc[0] < 1.0e-4);

  // TODO(russt): make a plot of the resulting trajectory (using vtk?).

  return 0;
}
