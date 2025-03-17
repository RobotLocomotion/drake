#include "drake/systems/primitives/zero_order_hold.h"

#include <utility>

namespace drake {
namespace systems {

template <typename T>
ZeroOrderHold<T>::ZeroOrderHold(
    double period_sec, double offset_sec, int vector_size,
    std::unique_ptr<const AbstractValue> abstract_model_value)
    : LeafSystem<T>(SystemTypeTag<ZeroOrderHold>()),
      period_sec_(period_sec),
      offset_sec_(offset_sec),
      abstract_model_value_(std::move(abstract_model_value)) {
  DRAKE_THROW_UNLESS(offset_sec >= 0.0);
  if (!is_abstract()) {
    DRAKE_DEMAND(vector_size != -1);
    // TODO(david-german-tri): remove the size parameter from the constructor
    // once #3109 supporting automatic sizes is resolved.
    BasicVector<T> model_value(vector_size);
    this->DeclareVectorInputPort("u", model_value);
    auto state_index = this->DeclareDiscreteState(vector_size);
    this->DeclarePeriodicDiscreteUpdateEvent(
        period_sec_, offset_sec_, &ZeroOrderHold::LatchInputVectorToState);
    this->DeclareStateOutputPort("y", state_index);
  } else {
    DRAKE_DEMAND(vector_size == -1);
    // TODO(eric.cousineau): Remove value parameter from the constructor once
    // the equivalent of #3109 for abstract values is also resolved.
    this->DeclareAbstractInputPort("u", *abstract_model_value_);
    auto state_index = this->DeclareAbstractState(*abstract_model_value_);
    this->DeclarePeriodicUnrestrictedUpdateEvent(
        period_sec_, offset_sec_,
        &ZeroOrderHold::LatchInputAbstractValueToState);
    this->DeclareStateOutputPort("y", state_index);
  }
}

template <typename T>
template <typename U>
ZeroOrderHold<T>::ZeroOrderHold(const ZeroOrderHold<U>& other)
    : ZeroOrderHold(other.period_sec_, other.offset_sec_,
                    other.is_abstract() ? -1 : other.get_input_port().size(),
                    other.is_abstract() ? other.abstract_model_value_->Clone()
                                        : nullptr) {}

template <typename T>
ZeroOrderHold<T>::~ZeroOrderHold() = default;

template <typename T>
void ZeroOrderHold<T>::SetVectorState(
    Context<T>* context, const Eigen::Ref<const VectorX<T>>& value) const {
  DRAKE_ASSERT(!is_abstract());
  this->ValidateContext(context);
  BasicVector<T>& state_vector = context->get_mutable_discrete_state_vector();
  // Asserts that the input value is a column vector of the appropriate size.
  DRAKE_THROW_UNLESS(value.rows() == state_vector.size());
  state_vector.SetFromVector(value);
}

template <typename T>
void ZeroOrderHold<T>::LatchInputVectorToState(
    const Context<T>& context, DiscreteValues<T>* discrete_state) const {
  DRAKE_ASSERT(!is_abstract());
  const auto& input = this->get_input_port().Eval(context);
  discrete_state->set_value(0, input);
}

template <typename T>
void ZeroOrderHold<T>::LatchInputAbstractValueToState(const Context<T>& context,
                                                      State<T>* state) const {
  DRAKE_ASSERT(is_abstract());
  const auto& input =
      this->get_input_port().template Eval<AbstractValue>(context);
  AbstractValue& state_value =
      state->get_mutable_abstract_state().get_mutable_value(0);
  state_value.SetFrom(input);
}

}  // namespace systems
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::systems::ZeroOrderHold);
