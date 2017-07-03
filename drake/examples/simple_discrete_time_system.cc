
// Simple Discrete Time System Example
//
// This is meant to be a sort of "hello world" example for the
// drake::system classes.  It defines a very simple discrete time system,
// simulates it from a given initial condition, and plots the result.

#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/vector_system.h"

// Simple Discrete Time System
//   x[n+1] = x[n]^3
//   y = x
class SimpleDiscreteTimeSystem : public drake::systems::VectorSystem<double> {
 public:
  SimpleDiscreteTimeSystem()
      : drake::systems::VectorSystem<double>(0, 1) {  // Zero inputs, 1 output.
    this->DeclarePeriodicDiscreteUpdate(1.0);
    this->DeclareDiscreteState(1);  // One state variable.
  }

 private:
  // x[n+1] = x[n]^3
  virtual void DoCalcVectorDiscreteVariableUpdates(
      const drake::systems::Context<double>& context,
      const Eigen::VectorBlock<const Eigen::VectorXd>& u,
      const Eigen::VectorBlock<const Eigen::VectorXd>& x,
      Eigen::VectorBlock<Eigen::VectorXd>* xn) const {
    (*xn)[0] = std::pow(x[0], 3.0);
  }

  // y = x
  virtual void DoCalcVectorOutput(
      const drake::systems::Context<double>& context,
      const Eigen::VectorBlock<const Eigen::VectorXd>& u,
      const Eigen::VectorBlock<const Eigen::VectorXd>& x,
      Eigen::VectorBlock<Eigen::VectorXd>* y) const {
    *y = x;
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
