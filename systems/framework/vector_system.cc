#include "drake/systems/framework/vector_system.h"

namespace drake {
namespace systems {

template <typename T>
VectorSystem<T>::~VectorSystem() = default;

template <typename T>
EventStatus VectorSystem<T>::CalcDiscreteUpdate(
    const Context<T>& context, DiscreteValues<T>* discrete_state) const {
  // Short-circuit when there's no work to do.
  if (discrete_state->num_groups() == 0) {
    return EventStatus::DidNothing();
  }

  const VectorX<T>& input_vector = EvalVectorInput(context);
  const auto input_block = input_vector.head(input_vector.rows());

  // Obtain the block form of xd before the update (i.e., the prior state).
  DRAKE_ASSERT(context.has_only_discrete_state());
  const VectorX<T>& state_vector = context.get_discrete_state(0).value();
  const Eigen::VectorBlock<const VectorX<T>> state_block =
      state_vector.head(state_vector.rows());

  // Obtain the block form of xd after the update (i.e., the next state).
  DRAKE_ASSERT(discrete_state != nullptr);
  Eigen::VectorBlock<VectorX<T>> discrete_update_block =
      discrete_state->get_mutable_value();

  // Delegate to subclass.
  DoCalcVectorDiscreteVariableUpdates(context, input_block, state_block,
                                      &discrete_update_block);

  return EventStatus::Succeeded();
}

}  // namespace systems
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::systems::VectorSystem);
