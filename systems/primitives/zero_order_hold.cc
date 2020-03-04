#include "drake/systems/primitives/zero_order_hold.h"

#include <utility>

namespace drake {
namespace systems {

template <typename T>
ZeroOrderHold<T>::ZeroOrderHold(
    double period_sec, int vector_size,
    std::unique_ptr<const AbstractValue> abstract_model_value)
    : LeafSystem<T>(SystemTypeTag<ZeroOrderHold>()),
      period_sec_(period_sec),
      abstract_model_value_(std::move(abstract_model_value)) {
  if (!is_abstract()) {
    DRAKE_DEMAND(vector_size != -1);
    // TODO(david-german-tri): remove the size parameter from the constructor
    // once #3109 supporting automatic sizes is resolved.
    BasicVector<T> model_value(vector_size);
    this->DeclareVectorInputPort("u", model_value);
    this->DeclareVectorOutputPort("y",
        model_value, &ZeroOrderHold::CopyLatchedVector,
        {this->xd_ticket()});
    this->DeclareDiscreteState(vector_size);
    this->DeclarePeriodicDiscreteUpdateEvent(period_sec_, 0.,
        &ZeroOrderHold::LatchInputVectorToState);
  } else {
    DRAKE_DEMAND(vector_size == -1);
    // TODO(eric.cousineau): Remove value parameter from the constructor once
    // the equivalent of #3109 for abstract values is also resolved.
    this->DeclareAbstractInputPort("u", *abstract_model_value_);

    // Because we're working with type-erased AbstractValue objects directly
    // (not typical), there isn't a nice sugar method available that takes
    // class methods. We have to use the generic port declaration method that
    // uses free functions.
    this->DeclareAbstractOutputPort("y",
        // Allocator function.
        [this]() { return abstract_model_value_->Clone(); },
        // Calculator function.
        [this](const Context<T>& context, AbstractValue* output) {
          this->CopyLatchedAbstractValue(context, &*output);
        },
        {this->xa_ticket()});

    this->DeclareAbstractState(abstract_model_value_->Clone());
    this->DeclarePeriodicUnrestrictedUpdateEvent(period_sec_, 0.,
        &ZeroOrderHold::LatchInputAbstractValueToState);
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
void ZeroOrderHold<T>::CopyLatchedVector(
      const Context<T>& context,
      BasicVector<T>* output) const {
  DRAKE_ASSERT(!is_abstract());
  const BasicVector<T>& state_value = context.get_discrete_state(0);
  output->SetFrom(state_value);
}

template <typename T>
void ZeroOrderHold<T>::LatchInputVectorToState(
    const Context<T>& context,
    DiscreteValues<T>* discrete_state) const {
  DRAKE_ASSERT(!is_abstract());
  const auto& input = get_input_port().Eval(context);
  BasicVector<T>& state_value = discrete_state->get_mutable_vector(0);
  state_value.SetFromVector(input);
}

template <typename T>
void ZeroOrderHold<T>::CopyLatchedAbstractValue(const Context<T>& context,
                                                AbstractValue* output) const {
  DRAKE_ASSERT(is_abstract());
  const AbstractValue& state_value = context.get_abstract_state().get_value(0);
  output->SetFrom(state_value);
}

template <typename T>
void ZeroOrderHold<T>::LatchInputAbstractValueToState(
    const Context<T>& context,
    State<T>* state) const {
  DRAKE_ASSERT(is_abstract());
  const auto& input = get_input_port().template Eval<AbstractValue>(context);
  AbstractValue& state_value =
      state->get_mutable_abstract_state().get_mutable_value(0);
  state_value.SetFrom(input);
}

}  // namespace systems
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::systems::ZeroOrderHold)
