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
void Integrator<T>::ReserveState(Context<T>* context) const {
  // The integrator has a state vector of the same dimension as its input
  // and output.
  DRAKE_ASSERT(System<T>::get_input_port(0).get_size() ==
      System<T>::get_output_port(0).get_size());
  int length = System<T>::get_input_port(0).get_size();
  context->get_mutable_state()->continuous_state.reset(new ContinuousState<T>(
      std::make_unique<BasicStateVector<T>>(length), 0 /* size of q */,
      0 /* size of v */, length /* size of z */));
}

template <typename T>
std::unique_ptr<ContinuousState<T>> Integrator<T>::AllocateTimeDerivatives()
    const {
  int length = System<T>::get_output_port(0).get_size();
  return std::make_unique<ContinuousState<T>>(
      std::make_unique<BasicStateVector<T>>(length), 0 /* size of q */,
      0 /* size of v */, length /* size of z */);
}

template <typename T>
void Integrator<T>::EvalOutput(const ContextBase<T>& context,
                               SystemOutput<T>* output) const {
  DRAKE_ASSERT(System<T>::IsValidOutput(*output));
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
