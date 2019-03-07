#include "drake/systems/primitives/discrete_time_delay.h"

#include <utility>

#include "drake/common/drake_copyable.h"

namespace drake {
namespace systems {

template <typename T>
DiscreteTimeDelay<T>::DiscreteTimeDelay(double update_sec, int delay_timesteps,
                                        int vector_size)
    : LeafSystem<T>(SystemTypeTag<systems::DiscreteTimeDelay>()),
      update_sec_(update_sec),
      delay_timesteps_(delay_timesteps),
      vector_size_(vector_size) {
  DRAKE_DEMAND(vector_size >= 0);
  // TODO(mpetersen94): remove the size parameter from the constructor
  // once #3109 supporting automatic sizes is resolved.
  BasicVector<T> model_value(vector_size);
  this->DeclareVectorInputPort("u", model_value);
  this->DeclareVectorOutputPort("delayed u", model_value,
                                &DiscreteTimeDelay::CopyDelayedVector);
  this->DeclareDiscreteState(vector_size * delay_timesteps);
  this->DeclarePeriodicDiscreteUpdateEvent(
      update_sec_, 0., &DiscreteTimeDelay::SaveInputVectorToBuffer);
}

template <typename T>
template <typename U>
DiscreteTimeDelay<T>::DiscreteTimeDelay(const DiscreteTimeDelay<U>& other)
    : DiscreteTimeDelay(other.update_sec_, other.delay_timesteps_,
                        other.vector_size_) {}

template <typename T>
void DiscreteTimeDelay<T>::CopyDelayedVector(
    const Context<T>& context, BasicVector<T>* output) const {
  const BasicVector<T>& state_value = context.get_discrete_state(0);
  output->SetFromVector(state_value.get_value().head(vector_size_));
}

template <typename T>
void DiscreteTimeDelay<T>::SaveInputVectorToBuffer(
    const Context<T>& context, DiscreteValues<T>* discrete_state) const {
  const auto& input = get_input_port().Eval(context);
  Eigen::VectorBlock<VectorX<T>> updated_state_value =
      discrete_state->get_mutable_vector(0).get_mutable_value();
  Eigen::VectorBlock<const VectorX<T>> old_state_value =
      context.get_discrete_state(0).get_value();
  updated_state_value.head((delay_timesteps_ - 1) * vector_size_) =
      old_state_value.tail((delay_timesteps_ - 1) * vector_size_);
  updated_state_value.tail(vector_size_) = input;
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
