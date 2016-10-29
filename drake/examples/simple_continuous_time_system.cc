
/// @file simple_continuous_time_system.cc
/// Simple Continuous Time System Example
///
/// This is meant to be a sort of "hello world" example for the
/// drake::system classes.  It defines a very simple continuous time system,
/// simulates it from a given initial condition, and plots the result.

#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/leaf_system.h"

using drake::systems::ContinuousState;

/// Simple Continuous Time System
/// @f[ \dot{x} = -x + x^3 @f]
/// @f[  y = x @f]
class SimpleContinuousTimeSystem : public drake::systems::LeafSystem<double> {
 public:
  SimpleContinuousTimeSystem() {
    const int kSize = 1;  // The dimension of both output (y) and state (x).
    this->DeclareOutputPort(drake::systems::kVectorValued,
                            kSize, drake::systems::kContinuousSampling);
    this->DeclareContinuousState(kSize);
  }
  ~SimpleContinuousTimeSystem() override{};

  // xdot = -x + x^3
  void EvalTimeDerivatives(
      const drake::systems::Context<double>& context,
      drake::systems::ContinuousState<double>* derivatives) const override {
    double x = context.get_continuous_state_vector().GetAtIndex(0);
    double xdot = -x + std::pow(x, 3.0);
    derivatives->get_mutable_vector()->SetAtIndex(0, xdot);
  }

  /// y = x
  void EvalOutput(const drake::systems::Context<double>& context,
                  drake::systems::SystemOutput<double>* output) const override {
    double x = context.get_continuous_state_vector().GetAtIndex(0);
    output->GetMutableVectorData(0)->SetAtIndex(0, x);
  }
};


int main(int argc, char* argv[]) {
  // create the simple system
  SimpleContinuousTimeSystem system;

  // create the simulator
  drake::systems::Simulator<double> simulator(system);

  // set the initial conditions x(0);
  ContinuousState<double>& xc =
      *simulator.get_mutable_context()->get_mutable_continuous_state();
  xc[0] = 0.9;

  // simulate for 10 seconds
  simulator.StepTo(10);

  // make sure the simulation converges to the stable fixed point at x=0
  DRAKE_ASSERT(xc[0] < 1.0e-4);

  // TODO(russt): make a plot of the resulting trajectory (using vtk?)

  return 0;
}
