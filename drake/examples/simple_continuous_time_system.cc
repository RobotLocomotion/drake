
/// Simple Continuous Time System Example
///
/// This is meant to be a sort of "hello world" example for the
/// drake::system classes.  It defines a very simple continous time system,
/// simulates it from a given initial condition, and plots the result.

#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace examples {

/// Simple Continuous Time System
/// \begin{gather*}
///     \dot{x} = -x + x^3 \\
///     y = x
/// \end{gather*}
class SimpleContinuousTimeSystem : public systems::LeafSystem<double> {
 public:
  SimpleContinuousTimeSystem() {
    this->DeclareOutputPort(systems::kVectorValued,
                            1,  // dimension of output (y) = 1
                            systems::kContinuousSampling);
  };
  ~SimpleContinuousTimeSystem() override{};

  // xdot = -x + x^3
  void EvalTimeDerivatives(
      const systems::Context<double>& context,
      systems::ContinuousState<double>* derivatives) const override {
    double x = context.get_continuous_state()->get_state().GetAtIndex(0);
    double xdot = -x + std::pow(x, 3.0);
    derivatives->get_mutable_state()->SetAtIndex(0, xdot);
  }

  /// y = x
  void EvalOutput(const systems::Context<double>& context,
                  systems::SystemOutput<double>* output) const override {
    double x = context.get_continuous_state()->get_state().GetAtIndex(0);
    output->GetMutableVectorData(0)->SetAtIndex(0, x);
  }

 protected:
  // allocate a basic vector of dimension 1
  virtual std::unique_ptr<systems::ContinuousState<double>>
  AllocateContinuousState() const override {
    return std::make_unique<systems::ContinuousState<double>>(
        std::make_unique<systems::BasicVector<double>>(1));
  }
};

}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  // create the simple system
  auto system = std::make_unique<drake::examples::SimpleContinuousTimeSystem>();

  // create the simulator
  drake::systems::Simulator<double> simulator(*system);

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
                            ->CopyToVector()
                            .value()) < 1.0e-4);

  // TODO(russt): make a plot of the resulting trajectory (using vtk?)

  return 0;
}
