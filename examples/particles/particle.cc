#include "drake/examples/particles/particle.h"

namespace drake {
namespace examples {
namespace particles {

template <typename T>
Particle<T>::Particle() {
  // A 1D input vector for acceleration.
  this->DeclareInputPort(systems::kUseDefaultName, systems::kVectorValued, 1);
  // Adding one generalized position and one generalized velocity.
  auto state_index = this->DeclareContinuousState(1, 1, 0);
  // A 2D output vector for position and velocity.
  this->DeclareStateOutputPort(systems::kUseDefaultName, state_index);
}

template <typename T>
void Particle<T>::DoCalcTimeDerivatives(
    const systems::Context<T>& context,
    systems::ContinuousState<T>* derivatives) const {
  // Get current state from context.
  const systems::VectorBase<T>& continuous_state_vector =
    context.get_continuous_state_vector();
  // Obtain the structure we need to write into.
  systems::VectorBase<T>& derivatives_vector =
    derivatives->get_mutable_vector();
  // Get current input acceleration value.
  const systems::BasicVector<T>* input_vector =
      this->EvalVectorInput(context, 0);
  // Set the derivatives. The first one is
  // velocity and the second one is acceleration.
  derivatives_vector.SetAtIndex(0, continuous_state_vector.GetAtIndex(1));
  derivatives_vector.SetAtIndex(1, input_vector->GetAtIndex(0));
}

template class Particle<double>;

}  // namespace particles
}  // namespace examples
}  // namespace drake
