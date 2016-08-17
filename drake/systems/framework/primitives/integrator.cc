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
Integrator<T>::Integrator(int length)
    : length_(length) {
  this->DeclareInputPort(kVectorValued, length, kContinuousSampling);
  this->DeclareOutputPort(kVectorValued, length, kContinuousSampling);
}

template <typename T>
void Integrator<T>::ReserveState(Context<T>* context) const {
  // The integrator has a state vector of the same dimension as its input
  // and output.
  context->get_mutable_state()->continuous_state.reset(new ContinuousState<T>(
      std::make_unique<BasicStateVector<T>>(length_), 0 /* size of q */,
      0 /* size of v */, length_ /* size of z */));
}

template <typename T>
std::unique_ptr<ContinuousState<T>> Integrator<T>::AllocateTimeDerivatives()
    const {
  return std::make_unique<ContinuousState<T>>(
      std::make_unique<BasicStateVector<T>>(length_), 0 /* size of q */,
      0 /* size of v */, length_ /* size of z */);
}

template <typename T>
void Integrator<T>::EvalOutput(const ContextBase<T>& context,
                               SystemOutput<T>* output) const {
  // Checks that the output is consistent with the definition of this system.
  DRAKE_ASSERT(System<T>::IsValidOutput(*output));

  // Checks that context is consistent with the definition of this system.
  DRAKE_ASSERT(System<T>::IsValidContext(context));

  VectorInterface<T>* output_port =
      output->get_mutable_port(0)->GetMutableVectorData();

  // TODO(david-german-tri): Remove this copy by allowing output ports to be
  // mere pointers to state variables (or cache lines).
  output_port->get_mutable_value() =
      context.get_state().continuous_state->get_state().CopyToVector();
}

template <typename T>
void Integrator<T>::EvalTimeDerivatives(const ContextBase<T>& context,
                                        ContinuousState<T>* derivatives) const {
  // Checks that context is consistent with the definition of this system.
  DRAKE_ASSERT(System<T>::IsValidContext(context));
  const VectorInterface<T>* input = context.get_vector_input(0);
  derivatives->get_mutable_state()->SetFromVector(input->get_value());
}

// Explicitly instantiates on the most common scalar types.
template class DRAKESYSTEMFRAMEWORK_EXPORT Integrator<double>;

}  // namespace systems
}  // namespace drake
