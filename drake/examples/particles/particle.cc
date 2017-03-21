///
/// @file   particle.cc
///
/// @brief Particle System implementation, plus instantiation with
/// commonly used scalar types.
///

#include "drake/examples/particles/particle.h"

namespace drake {
namespace examples {
namespace particles {

template <typename T>
Particle<T>::Particle() {
  // A 1d vector input for acceleration
  this->DeclareInputPort(systems::kVectorValued, 1);
  // Adding one generalized position and one generalized velocity
  this->DeclareContinuousState(1, 1, 0);
  // A 2d vector output for position and velocity
  this->DeclareOutputPort(systems::kVectorValued, 2);
}

template <typename T>
void Particle<T>::DoCalcOutput(
  const systems::Context<T>& context,
  systems::SystemOutput<T>* output) const {
  // Get current state from context.
  const systems::VectorBase<T>& continuous_state_vector =
      context.get_continuous_state_vector();
  // Obtain the structure we need to write into.
  systems::BasicVector<T>* const output_vector =
    output->GetMutableVectorData(0);
  // Write output
  output_vector->set_value(continuous_state_vector.CopyToVector());
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
  // Get current acceleration input value
  const systems::BasicVector<T>* input_vector =
      this->EvalVectorInput(context, 0);
  // Set the derivatives. The first one is the
  // velocity and the other one is the acceleration
  derivatives_vector->SetAtIndex(0, continuous_state_vector.GetAtIndex(1));
  derivatives_vector->SetAtIndex(1, input_vector->GetAtIndex(0));
}

template class Particle<double>;

}  // namespace particles
}  // namespace examples
}  // namespace drake
