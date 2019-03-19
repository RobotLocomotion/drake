#include "drake/systems/primitives/discrete_time_delay.h"

#include <utility>
#include <vector>

#include "drake/common/default_scalars.h"

namespace drake {
namespace systems {

struct AbstractValueBuffer {
  int current = 0;
  drake::copyable_unique_ptr<drake::systems::AbstractValues> buffer;
};

template <typename T>
DiscreteTimeDelay<T>::DiscreteTimeDelay(
    double update_sec, int delay_timesteps, int vector_size,
    std::unique_ptr<const AbstractValue> abstract_model_value)
    : LeafSystem<T>(SystemTypeTag<systems::DiscreteTimeDelay>()),
      update_sec_(update_sec),
      // Delay buffer must be one element longer to properly delay signal
      delay_buffer_size_(delay_timesteps + 1),
      vector_size_(vector_size),
      abstract_model_value_(std::move(abstract_model_value)) {
  if (!is_abstract()) {
    DRAKE_DEMAND(vector_size >= 0);
    // TODO(mpetersen94): remove the size parameter from the constructor
    // once #3109 supporting automatic sizes is resolved.
    BasicVector<T> model_value(vector_size);
    this->DeclareVectorInputPort("u", model_value);
    this->DeclareVectorOutputPort("delayed_u", model_value,
                                  &DiscreteTimeDelay::CopyDelayedVector);
    this->DeclareDiscreteState(vector_size_ * delay_buffer_size_);
    this->DeclarePeriodicDiscreteUpdateEvent(
        update_sec_, 0., &DiscreteTimeDelay::SaveInputVectorToBuffer);
  } else {
    DRAKE_DEMAND(vector_size < 0);
    // TODO(mpetersen94): Remove value parameter from the constructor once
    // the equivalent of #3109 for abstract values is also resolved.
    std::vector<std::unique_ptr<AbstractValue>> data;
    data.reserve(delay_buffer_size_);
    for (int ii = 0; ii < delay_buffer_size_; ii++) {
      data.emplace_back(abstract_model_value_->Clone());
    }
    AbstractValueBuffer model_state;
    model_state.buffer = std::make_unique<AbstractValues>(std::move(data));
    this->DeclareAbstractInputPort("u", *abstract_model_value_);
    this->DeclareAbstractOutputPort("delayed_u",
        [this]() { return abstract_model_value_->Clone(); },
        [this](const Context<T>& context, AbstractValue* out) {
          this->CopyDelayedAbstractValue(context, out);
        });
    this->DeclareAbstractState(AbstractValue::Make(model_state));
    this->DeclarePeriodicUnrestrictedUpdateEvent(
        update_sec_, 0., &DiscreteTimeDelay::SaveInputAbstractValueToBuffer);
  }
}

template <typename T>
template <typename U>
DiscreteTimeDelay<T>::DiscreteTimeDelay(const DiscreteTimeDelay<U>& other)
    : DiscreteTimeDelay(
          other.update_sec_, other.delay_buffer_size_ - 1, other.vector_size_,
          other.is_abstract() ? other.abstract_model_value_->Clone()
                              : nullptr) {}

template <typename T>
void DiscreteTimeDelay<T>::CopyDelayedVector(
    const Context<T>& context, BasicVector<T>* output) const {
  DRAKE_ASSERT(!is_abstract());
  const BasicVector<T>& state_value = context.get_discrete_state(0);
  output->SetFromVector(state_value.get_value().head(vector_size_));
}

template <typename T>
void DiscreteTimeDelay<T>::SaveInputVectorToBuffer(
    const Context<T>& context, DiscreteValues<T>* discrete_state) const {
  DRAKE_ASSERT(!is_abstract());
  // Copies the current state of the delay buffer from the context to the
  // discrete state, sliding the buffer forward one step, dropping the oldest
  // value and adding the value on the input port to the end of the buffer.
  // TODO(mpetersen94): consider revising to avoid possibly expensive buffer
  // copy operation.
  const auto& input = get_input_port().Eval(context);
  Eigen::VectorBlock<VectorX<T>> updated_state_value =
      discrete_state->get_mutable_vector(0).get_mutable_value();
  Eigen::VectorBlock<const VectorX<T>> old_state_value =
      context.get_discrete_state(0).get_value();
  updated_state_value.head((delay_buffer_size_ - 1) * vector_size_) =
      old_state_value.tail((delay_buffer_size_ - 1) * vector_size_);
  updated_state_value.tail(vector_size_) = input;
}

template <typename T>
void DiscreteTimeDelay<T>::CopyDelayedAbstractValue(
    const Context<T>& context, AbstractValue* output) const {
  DRAKE_ASSERT(is_abstract());
  const AbstractValueBuffer& state_value =
      context.template get_abstract_state<AbstractValueBuffer>(0);
  output->SetFrom(state_value.buffer->get_value(state_value.current));
}

template <typename T>
void DiscreteTimeDelay<T>::SaveInputAbstractValueToBuffer(
    const Context<T>& context, State<T>* state) const {
  DRAKE_ASSERT(is_abstract());
  const auto& input = get_input_port().template Eval<AbstractValue>(context);
  AbstractValueBuffer& state_value =
      state->template get_mutable_abstract_state<AbstractValueBuffer>(0);
  AbstractValue& updated_state_value =
      state_value.buffer->get_mutable_value(state_value.current);
  updated_state_value.SetFrom(input);
  state_value.current = (state_value.current + 1) % delay_buffer_size_;
}

template <typename T>
optional<bool> DiscreteTimeDelay<T>::DoHasDirectFeedthrough(
    int input_port, int output_port) const {
  DRAKE_DEMAND(input_port == 0);
  DRAKE_DEMAND(output_port == 0);
  // By definition, a time delay will not have direct feedthrough, as the
  // output only depends on the state, not the input.
  return false;
}

template <typename T>
std::unique_ptr<AbstractValues> DiscreteTimeDelay<T>::GetAbstractBuffer(
    Context<T>* context) {
  DRAKE_ASSERT(is_abstract());
  const AbstractValueBuffer buffer_state =
      context->template get_abstract_state<AbstractValueBuffer>(0);
  std::vector<std::unique_ptr<AbstractValue>> buffer_data;
  buffer_data.reserve(delay_buffer_size_);
  for (int ii = 0; ii < delay_buffer_size_; ii++) {
    const AbstractValue& next_value = buffer_state.buffer->get_value(
        (buffer_state.current + ii) % delay_buffer_size_);
    buffer_data.emplace_back(next_value.Clone());
  }
  return std::make_unique<AbstractValues>(std::move(buffer_data));
}

template <typename T>
void DiscreteTimeDelay<T>::SetAbstractBuffer(
    Context<T>* context, AbstractValues* updated_buffer_state) {
  DRAKE_ASSERT(is_abstract());
  DRAKE_ASSERT(updated_buffer_state->size() == delay_buffer_size_);
  AbstractValueBuffer& old_buffer_state =
      context->template get_mutable_abstract_state<AbstractValueBuffer>(0);
  old_buffer_state.current = 0;
  for (int ii = 0; ii < delay_buffer_size_; ii++) {
    AbstractValue& next_value = old_buffer_state.buffer->get_mutable_value(ii);
    next_value.SetFrom(updated_buffer_state->get_value(ii));
  }
}

}  // namespace systems
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::systems::DiscreteTimeDelay)
