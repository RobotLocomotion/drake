
/// @file simple_continuous_time_system.cc
/// Simple Continuous Time System Example
///
/// This is meant to be a sort of "hello world" example for the
/// drake::system classes.  It defines a very simple continuous time system,
/// simulates it from a given initial condition, and plots the result.

#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/leaf_system.h"

/// Simple Continuous Time System
/// \begin{gather*}
///     \dot{x} = -x + x^3 \newline
///     y = x
/// \end{gather*}
class SimpleContinuousTimeSystem : public drake::systems::LeafSystem<double> {
 public:
  SimpleContinuousTimeSystem() {
    this->DeclareOutputPort(drake::systems::kVectorValued,
                            1,  // dimension of output (y) = 1
                            drake::systems::kContinuousSampling);
  }
  ~SimpleContinuousTimeSystem() override{};

  // xdot = -x + x^3
  void EvalTimeDerivatives(
      const drake::systems::Context<double>& context,
      drake::systems::ContinuousState<double>* derivatives) const override {
    double x = context.get_continuous_state()->get_state().GetAtIndex(0);
    double xdot = -x + std::pow(x, 3.0);
    derivatives->get_mutable_state()->SetAtIndex(0, xdot);
  }

  /// y = x
  void EvalOutput(const drake::systems::Context<double>& context,
                  drake::systems::SystemOutput<double>* output) const override {
    double x = context.get_continuous_state()->get_state().GetAtIndex(0);
    output->GetMutableVectorData(0)->SetAtIndex(0, x);
  }

 protected:
  // allocate a basic vector of dimension 1
  std::unique_ptr<drake::systems::ContinuousState<double>>
  AllocateContinuousState() const override {
    return std::make_unique<drake::systems::ContinuousState<double>>(
        std::make_unique<drake::systems::BasicVector<double>>(1));
  }
};


int main(int argc, char* argv[]) {
  // create the simple system
  SimpleContinuousTimeSystem system;

  // create the simulator
  drake::systems::Simulator<double> simulator(system);

  // set the initial conditions x(0);
  auto initial_conditions = simulator.get_mutable_context();
  initial_conditions->get_mutable_continuous_state()
      ->get_mutable_state()
      ->SetAtIndex(0, .9);

  // simulate for 10 seconds
  simulator.StepTo(10);

  // make sure the simulation converges to the stable fixed point at x=0
  DRAKE_ASSERT(std::abs(simulator.get_context()
                            .get_continuous_state()
                            ->get_state().GetAtIndex(0)) < 1.0e-4);

  // TODO(russt): make a plot of the resulting trajectory (using vtk?)

  return 0;
}
