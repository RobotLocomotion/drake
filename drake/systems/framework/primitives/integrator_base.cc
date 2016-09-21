#include "drake/systems/framework/primitives/integrator.h"

#include <stdexcept>
#include <string>

#include "drake/common/drake_assert.h"
#include "drake/drakeSystemFramework_export.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/leaf_context.h"

namespace drake {
namespace systems {

template <typename T>
Integrator<T>::Integrator(int length) {
  this->DeclareInputPort(kVectorValued, length, kContinuousSampling);
  this->DeclareOutputPort(kVectorValued, length, kContinuousSampling);
}

template <typename T>
Integrator<T>::~Integrator() {}

template <typename T>
void Integrator<T>::set_integral_value(
    Context<T>* context, const Eigen::Ref<const VectorX<T>>& value) const {
  // TODO(amcastro-tri): Provide simple accessors here to avoid lengthy
  // constructions.
  auto state_vector =
      context->get_mutable_state()->continuous_state->get_mutable_state();
  // Asserts that the input value is a column vector of the appropriate size.
  DRAKE_ASSERT(value.rows() == state_vector->size() && value.cols() == 1);
  context->get_mutable_state()->continuous_state->
      get_mutable_state()->SetFromVector(value);
}

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
      std::make_unique<BasicVector<T>>(length));
}

template <typename T>
void Integrator<T>::EvalTimeDerivatives(const Context<T>& context,
                                        ContinuousState<T>* derivatives) const {
  DRAKE_ASSERT_VOID(System<T>::CheckValidContext(context));
  const BasicVector<T>* input = context.get_vector_input(0);
  derivatives->get_mutable_state()->SetFromVector(input->get_value());
}

template <typename T>
void Integrator<T>::EvalOutput(const Context<T>& context,
                               SystemOutput<T>* output) const {
  DRAKE_ASSERT_VOID(System<T>::CheckValidOutput(output));
  DRAKE_ASSERT_VOID(System<T>::CheckValidContext(context));

  // TODO(david-german-tri): Remove this copy by allowing output ports to be
  // mere pointers to state variables (or cache lines).
  System<T>::GetMutableOutputVector(output, 0) =
      System<T>::CopyContinuousStateVector(context);
}


// Explicitly instantiates on the most common scalar types.
template class DRAKESYSTEMFRAMEWORK_EXPORT Integrator<double>;

}  // namespace systems
}  // namespace drake
