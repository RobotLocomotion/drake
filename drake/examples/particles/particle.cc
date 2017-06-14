#include "drake/examples/particles/particle.h"

namespace drake {
namespace examples {
namespace particles {

template <typename T>
Particle<T>::Particle() {
  // A 1D input vector for acceleration.
  this->DeclareInputPort(systems::kVectorValued, 1);
  // Adding one generalized position and one generalized velocity.
  this->DeclareContinuousState(1, 1, 0);
  // A 2D output vector for position and velocity.
  this->DeclareVectorOutputPort(systems::BasicVector<T>(2),
                                &Particle::CopyStateOut);
}

template <typename T>
void Particle<T>::CopyStateOut(
    const systems::Context<T>& context,
    systems::BasicVector<T>* output) const {
  // Get current state from context.
  const systems::VectorBase<T>& continuous_state_vector =
      context.get_continuous_state_vector();
  // Write system output.
  output->set_value(continuous_state_vector.CopyToVector());
}

template <typename T>
void Particle<T>::DoCalcTimeDerivatives(
    const systems::Context<T>& context,
    systems::ContinuousState<T>* derivatives) const {
  // Get current state from context.
  const systems::VectorBase<T>& continuous_state_vector =
    context.get_continuous_state_vector();
  // Obtain the structure we need to write into.
  systems::VectorBase<T>* const derivatives_vector =
    derivatives->get_mutable_vector();
  // Get current input acceleration value.
  const systems::BasicVector<T>* input_vector =
      this->EvalVectorInput(context, 0);
  // Set the derivatives. The first one is
  // velocity and the second one is acceleration.
  derivatives_vector->SetAtIndex(0, continuous_state_vector.GetAtIndex(1));
  derivatives_vector->SetAtIndex(1, input_vector->GetAtIndex(0));
}

template class Particle<double>;

}  // namespace particles
}  // namespace examples
}  // namespace drake
