// Simple Mixed Continuous-Time/Discrete-Time System Example
//
// This is meant to be a sort of "hello world" example for the
// drake::system classes.  It defines a very simple system with both continuous
// and discrete time dynamics, simulates it from a given initial condition, and
// checks the result.

#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/leaf_system.h"

// Simple Discrete Time System
//   xd_{n+1} =  xd_n³
//   xcdot    = -xc + xc³
//   y        = [xd;xc]
class SimpleMixedContinuousTimeDiscreteTimeSystem
    : public drake::systems::LeafSystem<double> {
 public:
  SimpleMixedContinuousTimeDiscreteTimeSystem() {
    const int kSize = 1;
    this->DeclarePeriodicDiscreteUpdateEvent(
        1.0, 0.0, &SimpleMixedContinuousTimeDiscreteTimeSystem::Update);
    this->DeclareVectorOutputPort(
        "y", drake::systems::BasicVector<double>(2 * kSize),
        &SimpleMixedContinuousTimeDiscreteTimeSystem::CopyStateOut);
    this->DeclareContinuousState(kSize);
    this->DeclareDiscreteState(kSize);
  }

 private:
  // xd_{n+1} =  xd_n³
  drake::systems::EventStatus Update(
      const drake::systems::Context<double>& context,
      drake::systems::DiscreteValues<double>* updates) const {
    const double x_n = context.get_discrete_state()[0];
    const double x_np1 = std::pow(x_n, 3.0);
    (*updates)[0] = x_np1;
    return drake::systems::EventStatus::Succeeded();
  }

  // xcdot = -xc + xc³
  void DoCalcTimeDerivatives(
      const drake::systems::Context<double>& context,
      drake::systems::ContinuousState<double>* derivatives) const override {
    const double xc = context.get_continuous_state()[0];
    const double xcdot = -xc + std::pow(xc, 3.0);
    (*derivatives)[0] = xcdot;
  }

  // y = x
  void CopyStateOut(const drake::systems::Context<double>& context,
                    drake::systems::BasicVector<double>* output) const {
    const double xd = context.get_discrete_state()[0];
    (*output)[0] = xd;

    const double xc = context.get_continuous_state_vector().GetAtIndex(0);
    (*output)[1] = xc;
  }
};

int main() {
  // Create the simple system.
  SimpleMixedContinuousTimeDiscreteTimeSystem system;

  // Create the simulator.
  drake::systems::Simulator<double> simulator(system);

  // Set the initial conditions xd₀, xc(0).
  drake::systems::DiscreteValues<double>& xd =
      simulator.get_mutable_context().get_mutable_discrete_state();
  xd[0] = 0.99;  // xd₀
  drake::systems::ContinuousState<double>& xc =
      simulator.get_mutable_context().get_mutable_continuous_state();
  xc[0] = 0.9;  // xc(0)

  // Simulate for 10 seconds.
  simulator.StepTo(10);

  // Make sure the simulation converges to the stable fixed point at x=0.
  DRAKE_DEMAND(xd[0] < 1.0e-4);
  DRAKE_DEMAND(xc[0] < 1.0e-4);

  // TODO(russt): make a plot of the resulting trajectory (using vtk?).

  return 0;
}
