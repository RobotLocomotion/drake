#pragma once

/// @file
/// Template method implementations for zero_order_hold.h.
/// Most users should only include that file, not this one.
/// For background, see http://drake.mit.edu/cxx_inl.html.

#include <memory>
#include <vector>

#include "drake/common/unused.h"
#include "drake/systems/primitives/zero_order_hold.h"

namespace drake {
namespace systems {

template <typename T>
ZeroOrderHold<T>::ZeroOrderHold(double period_sec, int size)
    : period_sec_(period_sec) {
  // TODO(david-german-tri): remove the size parameter from the constructor
  // once #3109 supporting automatic sizes is resolved.
  BasicVector<T> dummy_value(size);
  this->DeclareVectorInputPort(dummy_value);
  this->DeclareVectorOutputPort(
      dummy_value, &ZeroOrderHold::DoCalcVectorOutput);
  this->DeclareDiscreteState(size);
  this->DeclarePeriodicDiscreteUpdate(period_sec);
}

template <typename T>
ZeroOrderHold<T>::ZeroOrderHold(double period_sec,
                                const AbstractValue& model_value)
    : period_sec_(period_sec), abstract_model_value_(model_value.Clone()) {
  // TODO(eric.cousineau): Remove value parameter from the constructor once
  // the equivalent of #3109 for abstract values is also resolved.
  this->DeclareAbstractInputPort(model_value);
  // We must bind these because the instance method pointer overload expects
  // the allocator to return a value-type `OutputType`, which would cause
  // unwanted slicing / casting to Value<AbstractValue>.
  namespace sp = std::placeholders;
  this->DeclareAbstractOutputPort(
      std::bind(&ZeroOrderHold::AllocateAbstractValue, this, sp::_1),
      std::bind(&ZeroOrderHold::DoCalcAbstractOutput, this, sp::_1, sp::_2));
  this->DeclareAbstractState(model_value.Clone());
  this->DeclarePeriodicUnrestrictedUpdate(period_sec, 0.);
}

template <typename T>
void ZeroOrderHold<T>::DoCalcVectorOutput(
      const Context<T>& context,
      BasicVector<T>* output) const {
  DRAKE_ASSERT(!is_abstract());
  const auto& state_value = *context.get_discrete_state(0);
  output->SetFrom(state_value);
}

template <typename T>
void ZeroOrderHold<T>::DoCalcDiscreteVariableUpdates(
    const Context <T>& context,
    const std::vector<const DiscreteUpdateEvent<T>*>&,
    DiscreteValues <T> *discrete_state) const {
  DRAKE_ASSERT(!is_abstract());
  const auto& input_value = *this->EvalVectorInput(context, 0);
  auto& state_value = *discrete_state->get_mutable_vector(0);
  state_value.SetFrom(input_value);
}

template <typename T>
std::unique_ptr<AbstractValue>
ZeroOrderHold<T>::AllocateAbstractValue(const Context<T>&) const {
  return abstract_model_value_->Clone();
}

template <typename T>
void ZeroOrderHold<T>::DoCalcAbstractOutput(const Context<T>& context,
                                            AbstractValue* output) const {
  DRAKE_ASSERT(is_abstract());
  // Do not use template get_abstract_state<U>, because this will attempt to
  // cast the value to Value<AbstractValue>. Instead, query the AbstractValue
  // pointer directly.
  const auto& state_value =
      context.template get_abstract_state()->get_value(0);
  output->SetFrom(state_value);
}

template <typename T>
void ZeroOrderHold<T>::DoCalcUnrestrictedUpdate(
    const Context<T>& context,
    const std::vector<const UnrestrictedUpdateEvent<T>*>&,
    State<T> *state) const {
  DRAKE_ASSERT(is_abstract());
  const auto& input_value = *this->EvalAbstractInput(context, 0);
  // See `DoCalcAbstractOutput` for rationale regarding non-templated value
  // accessor.
  auto& state_value =
      state->get_mutable_abstract_state()->get_mutable_value(0);
  state_value.SetFrom(input_value);
}

template <typename T>
ZeroOrderHold<symbolic::Expression>* ZeroOrderHold<T>::DoToSymbolic() const {
  if (!is_abstract()) {
    return new ZeroOrderHold<symbolic::Expression>(
        period_sec_, this->get_input_port().size());
  } else {
    // Transmographication not supported for abstract values.
    throw std::runtime_error(
        "DoToSymbolic not implemented for abstract values.");
  }
}

}  // namespace systems
}  // namespace drake
