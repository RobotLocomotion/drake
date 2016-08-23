#include "drake/systems/framework/primitives/integrator.h"

#include <stdexcept>
#include <string>

#include "drake/common/drake_assert.h"
#include "drake/drakeSystemFramework_export.h"
#include "drake/systems/framework/basic_state_vector.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/context.h"

namespace drake {
namespace systems {

template <typename T>
Integrator<T>::Integrator(int length) {
  this->DeclareInputPort(kVectorValued, length, kContinuousSampling);
  this->DeclareOutputPort(kVectorValued, length, kContinuousSampling);
}

template <typename T>
Integrator<T>::~Integrator() {}

// TODO(amcastro-tri): we should be able to express that initial conditions
// feed through an integrator but the dynamic signal during simulation does
// not.
template <typename T>
bool Integrator<T>::has_any_direct_feedthrough() const {
  return false;
}

template <typename T>
std::unique_ptr<ContinuousState<T>> Integrator<T>::AllocateContinuousState()
    const {
  // The integrator's state is first-order; its state vector length is the
  // same as the input (and output) vector length.
  const int length = System<T>::get_output_port(0).get_size();
  DRAKE_ASSERT(System<T>::get_input_port(0).get_size() == length);
  return std::make_unique<ContinuousState<T>>(
      std::make_unique<BasicStateVector<T>>(length));
}

template <typename T>
void Integrator<T>::EvalTimeDerivatives(const ContextBase<T>& context,
                                        ContinuousState<T>* derivatives) const {
  DRAKE_ASSERT_VOID(System<T>::CheckValidContext(context));
  const VectorBase<T>* input = context.get_vector_input(0);
  derivatives->get_mutable_state()->SetFromVector(input->get_value());
}

template <typename T>
void Integrator<T>::EvalOutput(const ContextBase<T>& context,
                               SystemOutput<T>* output) const {
  DRAKE_ASSERT_VOID(System<T>::CheckValidOutput(output));
  DRAKE_ASSERT_VOID(System<T>::CheckValidContext(context));

  VectorBase<T>* output_port = output->GetMutableVectorData(0);

  // TODO(david-german-tri): Remove this copy by allowing output ports to be
  // mere pointers to state variables (or cache lines).
  output_port->get_mutable_value() =
      context.get_state().continuous_state->get_state().CopyToVector();
}


// Explicitly instantiates on the most common scalar types.
template class DRAKESYSTEMFRAMEWORK_EXPORT Integrator<double>;

}  // namespace systems
}  // namespace drake
