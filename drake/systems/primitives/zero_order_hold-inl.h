#pragma once

/// @file
/// Template method implementations for zero_order_hold.h.
/// Most users should only include that file, not this one.
/// For background, see http://drake.mit.edu/cxx_inl.html.

/* clang-format off to disable clang-format-includes */
#include "drake/systems/primitives/zero_order_hold.h"
/* clang-format on */

#include <memory>
#include <utility>
#include <vector>

namespace drake {
namespace systems {

template <typename T>
ZeroOrderHold<T>::ZeroOrderHold(
    double period_sec, int vector_size,
    std::unique_ptr<const AbstractValue> abstract_model_value)
    : LeafSystem<T>(SystemTypeTag<systems::ZeroOrderHold>()),
      period_sec_(period_sec),
      abstract_model_value_(std::move(abstract_model_value)) {
  if (!is_abstract()) {
    DRAKE_DEMAND(vector_size != -1);
    // TODO(david-german-tri): remove the size parameter from the constructor
    // once #3109 supporting automatic sizes is resolved.
    BasicVector<T> model_value(vector_size);
    this->DeclareVectorInputPort(model_value);
    this->DeclareVectorOutputPort(
        model_value, &ZeroOrderHold::DoCalcVectorOutput);
    this->DeclareDiscreteState(vector_size);
    this->DeclarePeriodicDiscreteUpdate(period_sec_);
  } else {
    DRAKE_DEMAND(vector_size == -1);
    // TODO(eric.cousineau): Remove value parameter from the constructor once
    // the equivalent of #3109 for abstract values is also resolved.
    this->DeclareAbstractInputPort(*abstract_model_value_);
    // Use the std::function<> overloads to work with `AbstractValue` type
    // directly and maintain type erasure.
    auto abstract_value_allocator = [this](const Context<T>&) {
      return abstract_model_value_->Clone();
    };
    namespace sp = std::placeholders;
    this->DeclareAbstractOutputPort(
        abstract_value_allocator,
        std::bind(&ZeroOrderHold::DoCalcAbstractOutput, this, sp::_1, sp::_2));
    this->DeclareAbstractState(abstract_model_value_->Clone());
    this->DeclarePeriodicUnrestrictedUpdate(period_sec_, 0.);
  }
}

template <typename T>
template <typename U>
ZeroOrderHold<T>::ZeroOrderHold(const ZeroOrderHold<U>& other)
    : ZeroOrderHold(other.period_sec_,
                    other.is_abstract() ? -1 : other.get_input_port().size(),
                    other.is_abstract() ? other.abstract_model_value_->Clone()
                                        : nullptr) {}

template <typename T>
void ZeroOrderHold<T>::DoCalcVectorOutput(
      const Context<T>& context,
      BasicVector<T>* output) const {
  DRAKE_ASSERT(!is_abstract());
  const BasicVector<T>& state_value = context.get_discrete_state(0);
  output->SetFrom(state_value);
}

template <typename T>
void ZeroOrderHold<T>::DoCalcDiscreteVariableUpdates(
    const Context<T>& context,
    const std::vector<const DiscreteUpdateEvent<T>*>&,
    DiscreteValues<T>* discrete_state) const {
  DRAKE_ASSERT(!is_abstract());
  const BasicVector<T>& input_value = *this->EvalVectorInput(context, 0);
  BasicVector<T>& state_value = discrete_state->get_mutable_vector(0);
  state_value.SetFrom(input_value);
}

template <typename T>
void ZeroOrderHold<T>::DoCalcAbstractOutput(const Context<T>& context,
                                            AbstractValue* output) const {
  DRAKE_ASSERT(is_abstract());
  // Do not use `get_abstract_state<AbstractValue>` since it would cast
  // the value to `Value<AbstractValue>`, which is an invalid type by design.
  const AbstractValue& state_value =
      context.template get_abstract_state().get_value(0);
  output->SetFrom(state_value);
}

template <typename T>
void ZeroOrderHold<T>::DoCalcUnrestrictedUpdate(
    const Context<T>& context,
    const std::vector<const UnrestrictedUpdateEvent<T>*>&,
    State<T>* state) const {
  DRAKE_ASSERT(is_abstract());
  const AbstractValue& input_value = *this->EvalAbstractInput(context, 0);
  // See `DoCalcAbstractOutput` for rationale regarding non-templated value
  // accessor.
  AbstractValue& state_value =
      state->get_mutable_abstract_state().get_mutable_value(0);
  state_value.SetFrom(input_value);
}

template <typename T>
optional<bool> ZeroOrderHold<T>::DoHasDirectFeedthrough(
    int input_port, int output_port) const {
  DRAKE_DEMAND(input_port == 0);
  DRAKE_DEMAND(output_port == 0);
  // By definition, a zero-order hold will not have direct feedthrough, as the
  // output only depends on the state, not the input.
  return false;
}

}  // namespace systems
}  // namespace drake
