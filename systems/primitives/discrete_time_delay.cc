#include "drake/systems/primitives/discrete_time_delay.h"

#include <utility>

#include "drake/common/drake_copyable.h"

namespace drake {
namespace systems {

template <typename T>
DiscreteTimeDelay<T>::DiscreteTimeDelay(
    double update_sec, int delay_timesteps, int vector_size,
    std::unique_ptr<const AbstractValue> abstract_model_value)
    : LeafSystem<T>(SystemTypeTag<systems::DiscreteTimeDelay>()),
      update_sec_(update_sec),
      delay_timesteps_(delay_timesteps),
      vector_size_(vector_size),
      abstract_model_value_(std::move(abstract_model_value)) {
  if (!is_abstract()) {
    DRAKE_DEMAND(vector_size != -1);
    // TODO(mpetersen94): remove the size parameter from the constructor
    // once #3109 supporting automatic sizes is resolved.
    BasicVector<T> model_value(vector_size);
    this->DeclareVectorInputPort(model_value);
    this->DeclareVectorOutputPort(model_value,
                                  &DiscreteTimeDelay::CalcVectorOutput);
    this->DeclareDiscreteState(vector_size * delay_timesteps);
    this->DeclarePeriodicUnrestrictedUpdate(update_sec_);
  } else {
    DRAKE_DEMAND(vector_size == -1);
    // // TODO(mpetersen94): Remove value parameter from the constructor once
    // // the equivalent of #3109 for abstract values is also resolved.
    // this->DeclareAbstractInputPort(*abstract_model_value_);
    // this->DeclareAbstractOutputPort(
    //     [this]() { return abstract_model_value_->Clone(); },
    //     [this](const Context<double>& context, AbstractValue* out) {
    //       this->CalcAbstractOutput(context, out);
    //     });
    // this->DeclareAbstractState(
    //     AbstractValue::Make(DelayBuffer<const AbstractValue>(
    //         delay_sec_ / update_sec_, abstract_model_value_->Clone())));
    // this->DeclarePeriodicUnrestrictedUpdate(update_sec_);
  }
}

template <typename T>
template <typename U>
DiscreteTimeDelay<T>::DiscreteTimeDelay(const DiscreteTimeDelay<U>& other)
    : DiscreteTimeDelay(
          other.update_sec_, other.delay_timesteps_, other.vector_size_,
          other.is_abstract() ? other.abstract_model_value_->Clone()
                              : nullptr) {}

template <typename T>
void DiscreteTimeDelay<T>::CalcVectorOutput(const Context<T>& context,
                                            BasicVector<T>* output) const {
  DRAKE_ASSERT(!is_abstract());
  const BasicVector<T>& state_value = context.get_discrete_state(0);
  output->SetFromVector(state_value.get_value().head(vector_size_));
}

template <typename T>
void DiscreteTimeDelay<T>::DoCalcDiscreteVariableUpdates(
    const Context<T>& context,
    const std::vector<const DiscreteUpdateEvent<T>*>&,
    DiscreteValues<T>* discrete_state) const {
  DRAKE_ASSERT(!is_abstract());
  const auto& input = get_input_port().Eval(context);
  Eigen::VectorBlock<VectorX<T>> updated_state_value =
      discrete_state->get_mutable_vector(0).get_mutable_value();
  Eigen::VectorBlock<const VectorX<T>> old_state_value =
      context.get_discrete_state(0).get_value();
  updated_state_value.head((delay_timesteps_ - 1) * vector_size_) =
      old_state_value.tail((delay_timesteps_ - 1) * vector_size_);
  updated_state_value.tail(vector_size_) = input;
}

// context, output
template <typename T>
void DiscreteTimeDelay<T>::CalcAbstractOutput(const Context<T>&,
                                              AbstractValue*) const {
  DRAKE_ASSERT(is_abstract());
  DRAKE_ASSERT(!is_abstract());  // AbstractValue not yet supported
  // const DelayBuffer<const AbstractValue> delay_buffer =
  //     context.get_abstract_state<DelayBuffer<const AbstractValue>>(0);
  // output->SetFrom(delay_buffer.GetDelayedValue());
}

// context, ~, state
template <typename T>
void DiscreteTimeDelay<T>::DoCalcUnrestrictedUpdate(
    const Context<T>&,
    const std::vector<const UnrestrictedUpdateEvent<T>*>&,
    State<T>*) const {
  DRAKE_ASSERT(is_abstract());
  DRAKE_ASSERT(!is_abstract());  // AbstractValue not yet supported
  // const auto& input = get_input_port().Eval(context);
  // DelayBuffer<const AbstractValue>& delay_buffer =
  //     state->get_mutable_abstract_state<DelayBuffer<const AbstractValue>>(
  //         0);
  // delay_buffer.UpdateBuffer(input);
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

}  // namespace systems
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::systems::DiscreteTimeDelay)
