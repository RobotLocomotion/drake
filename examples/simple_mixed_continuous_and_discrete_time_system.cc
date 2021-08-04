// Simple Mixed Continuous-Time/Discrete-Time System Example
//
// This is meant to be a sort of "hello world" example for the
// drake::system classes.  It defines a very simple system with both continuous
// and discrete time dynamics, simulates it from a given initial condition, and
// checks the result.

#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace systems {
namespace {

// Simple Discrete Time System
//   xd_{n+1} =  xd_n³
//   xcdot    = -xc + xc³
//   y        = [xd;xc]
class SimpleMixedContinuousTimeDiscreteTimeSystem : public LeafSystem<double> {
 public:
  SimpleMixedContinuousTimeDiscreteTimeSystem() {
    DeclarePeriodicDiscreteUpdateEvent(
        1.0, 0.0, &SimpleMixedContinuousTimeDiscreteTimeSystem::Update);
    DeclareVectorOutputPort(
        "y", 2,  // xd;xc
        &SimpleMixedContinuousTimeDiscreteTimeSystem::CopyStateOut);
    DeclareDiscreteState(1);    // xd
    DeclareContinuousState(1);  // xc
  }

 private:
  // xd_{n+1} =  xd_n³
  void Update(const Context<double>& context,
              DiscreteValues<double>* updates) const {
    const double xd_n = context.get_discrete_state()[0];
    const double xd_np1 = std::pow(xd_n, 3.0);
    (*updates)[0] = xd_np1;
  }

  // xcdot = -xc + xc³
  void DoCalcTimeDerivatives(
      const Context<double>& context,
      ContinuousState<double>* derivatives) const override {
    const double xc = context.get_continuous_state()[0];
    const double xcdot = -xc + std::pow(xc, 3.0);
    (*derivatives)[0] = xcdot;
  }

  // y = x
  void CopyStateOut(const Context<double>& context,
                    BasicVector<double>* output) const {
    const double xd = context.get_discrete_state()[0];
    (*output)[0] = xd;

    const double xc = context.get_continuous_state()[0];
    (*output)[1] = xc;
  }
};

int main() {
  // Create the simple system.
  SimpleMixedContinuousTimeDiscreteTimeSystem system;

  // Create the simulator.
  Simulator<double> simulator(system);

  // Set the initial conditions xd_0, xc(0).
  DiscreteValues<double>& xd =
      simulator.get_mutable_context().get_mutable_discrete_state();
  xd[0] = 0.99;  // xd_0
  ContinuousState<double>& xc =
      simulator.get_mutable_context().get_mutable_continuous_state();
  xc[0] = 0.9;  // xc(0)

  // Simulate for 10 seconds.
  simulator.AdvanceTo(10);

  // Make sure the simulation converges to the stable fixed point at x=0.
  DRAKE_DEMAND(xd[0] < 1.0e-4);
  DRAKE_DEMAND(xc[0] < 1.0e-4);

  // TODO(russt): make a plot of the resulting trajectory (using vtk?).

  return 0;
}

}  // namespace
}  // namespace systems
}  // namespace drake

int main() {
  return drake::systems::main();
}
