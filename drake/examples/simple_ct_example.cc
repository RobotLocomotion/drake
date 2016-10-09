
#include <iostream>

#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace examples {

/// Simple Continuous Time System Example
///     \dot{x} = -x + x^3
///     y = x
template <typename T>
class SimpleCTSystem : public systems::LeafSystem<T> {
 public:
  SimpleCTSystem() {
    this->DeclareOutputPort(systems::kVectorValued,
                            1,  // dimension of output (y) = 1
                            systems::kContinuousSampling);
  };
  ~SimpleCTSystem() override{};

  /// the output does not depend on any of the inputs directly (because there
  /// are no inputs to this system)
  bool has_any_direct_feedthrough() const override { return false; }

  /// y = x
  void EvalOutput(const systems::Context<T>& context,
                  systems::SystemOutput<T>* output) const override {
    output->GetMutableVectorData(0)->set_value(
        context.get_continuous_state()->get_state().CopyToVector());
    std::cout << "y(" << context.get_time() << ") = "
              << context.get_continuous_state()->get_state().GetAtIndex(0)
              << std::endl;
  }

  // xdot = -x + x^3
  void EvalTimeDerivatives(
      const systems::Context<T>& context,
      systems::ContinuousState<T>* derivatives) const override {
    DRAKE_ASSERT_VOID(systems::System<T>::CheckValidContext(context));

    const T x = context.get_continuous_state()->get_state().GetAtIndex(0);
    T xdot = -x + std::pow(x, 3.0);
    derivatives->get_mutable_state()->SetAtIndex(0, xdot);
    std::cout << "x(" << context.get_time() << ") = " << x << std::endl;
  }

 protected:
  // allocate a basic vector of dimension 1
  virtual std::unique_ptr<systems::ContinuousState<T>> AllocateContinuousState()
      const override {
    return std::make_unique<systems::ContinuousState<T>>(
        std::make_unique<systems::BasicVector<T>>(1));
  }
};

}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  // create the simple system
  auto system = std::make_unique<drake::examples::SimpleCTSystem<double>>();

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
