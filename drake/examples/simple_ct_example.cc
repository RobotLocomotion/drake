
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
    this->DeclareOutputPort(
        systems::kVectorValued,
        1,  // dimension of output (y) = 1
        systems::kContinuousSampling);
  };
  ~SimpleCTSystem() override {};

  /// the output does not depend on any of the inputs directly (because there
  /// are no inputs to this system)
  bool has_any_direct_feedthrough() const override { return false; }

  /// y = x
  void EvalOutput(const systems::Context<T>& context, systems::SystemOutput<T>*
  output) const override
  {
    output->GetMutableVectorData(0)->set_value(context.get_continuous_state()
                                                   ->get_state().CopyToVector());
  }

  // xdot = -x + x^3
  void EvalTimeDerivatives(const systems::Context<T>& context,
                           systems::ContinuousState<T>* derivatives) const override
  {
    DRAKE_ASSERT_VOID(systems::System<T>::CheckValidContext(context));

    const T x = context.get_continuous_state()->get_state().GetAtIndex(0);
    T xdot = -x + std::pow(x,3.0);
    derivatives->get_mutable_state()->SetAtIndex(0,xdot);
  }

 protected:
  // allocate a basic vector of dimension 1
  virtual std::unique_ptr<systems::ContinuousState<T> >
  AllocateContinuousState()
  const override {
    return std::make_unique<systems::ContinuousState<T>>(
        std::make_unique<systems::BasicVector<T>>(1));
  }
};


}  // namespace examples
}  // namespace drake


int main(int argc, char* argv[])
{
  // create the simple system
  auto system = std::make_unique<drake::examples::SimpleCTSystem<double>>();

  drake::systems::Simulator<double> simulator(*system);

  // set the initial conditions to x = .1;
  auto initial_conditions = simulator.get_mutable_context();
  initial_conditions->get_mutable_continuous_state()->get_mutable_state()
      ->SetAtIndex(0,.1);

  // simulate for 10 seconds
  simulator.Initialize();
  simulator.StepTo(10);

  return 0;
}

