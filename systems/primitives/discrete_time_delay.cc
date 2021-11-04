#include "drake/systems/primitives/discrete_time_delay.h"

#include <utility>
#include <vector>

#include "drake/common/default_scalars.h"

namespace drake {
namespace systems {

template <typename T>
DiscreteTimeDelay<T>::DiscreteTimeDelay(
    double update_sec, int delay_timesteps, int vector_size,
    std::unique_ptr<const AbstractValue> abstract_model_value)
    : LeafSystem<T>(SystemTypeTag<DiscreteTimeDelay>()),
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
                                  &DiscreteTimeDelay::CopyDelayedVector,
                                  {this->xd_ticket()});
    this->DeclareDiscreteState(vector_size_ * delay_buffer_size_);
    this->DeclarePeriodicDiscreteUpdateEvent(
        update_sec_, 0., &DiscreteTimeDelay::SaveInputVectorToBuffer);
  } else {
    DRAKE_DEMAND(vector_size < 0);
    // TODO(mpetersen94): Remove value parameter from the constructor once
    // the equivalent of #3109 for abstract values is also resolved.
    this->DeclareAbstractInputPort("u", *abstract_model_value_);
    this->DeclareAbstractOutputPort("delayed_u",
        [this]() { return abstract_model_value_->Clone(); },
        [this](const Context<T>& context, AbstractValue* out) {
          this->CopyDelayedAbstractValue(context, out);
        },
        {this->xa_ticket()});
    for (int ii = 0; ii < delay_buffer_size_; ++ii) {
      this->DeclareAbstractState(*abstract_model_value_);
    }
    // This state keeps track of the index of the oldest value in the buffer.
    // It is at abstract state index `delay_buffer_size_`.
    this->DeclareAbstractState(Value<int>(0));
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
  const auto& input = this->get_input_port().Eval(context);
  Eigen::VectorBlock<VectorX<T>> updated_state_value =
      discrete_state->get_mutable_value(0);
  const VectorX<T>& old_state_value = context.get_discrete_state(0).value();
  updated_state_value.head((delay_buffer_size_ - 1) * vector_size_) =
      old_state_value.tail((delay_buffer_size_ - 1) * vector_size_);
  updated_state_value.tail(vector_size_) = input;
}

template <typename T>
void DiscreteTimeDelay<T>::CopyDelayedAbstractValue(
    const Context<T>& context, AbstractValue* output) const {
  DRAKE_ASSERT(is_abstract());
  const int& oldest_index =
      context.template get_abstract_state<int>(delay_buffer_size_);
  const AbstractValue& state_value =
      context.get_abstract_state().get_value(oldest_index);
  output->SetFrom(state_value);
}

template <typename T>
void DiscreteTimeDelay<T>::SaveInputAbstractValueToBuffer(
    const Context<T>& context, State<T>* state) const {
  DRAKE_ASSERT(is_abstract());
  const auto& input =
      this->get_input_port().template Eval<AbstractValue>(context);
  int& oldest_index =
      state->template get_mutable_abstract_state<int>(delay_buffer_size_);
  AbstractValue& state_value =
      state->get_mutable_abstract_state().get_mutable_value(oldest_index);
  state_value.SetFrom(input);
  oldest_index = (oldest_index + 1) % delay_buffer_size_;
}

}  // namespace systems
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::systems::DiscreteTimeDelay)
