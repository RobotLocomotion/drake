#include "drake/multibody/plant/desired_state_input.h"

#include "drake/common/default_scalars.h"

namespace drake {
namespace multibody {
namespace internal {

template <typename T>
DesiredStateInput<T>::DesiredStateInput(int num_model_instances) {
  positions_.resize(num_model_instances);
  velocities_.resize(num_model_instances);
}

template <typename T>
DesiredStateInput<T>::~DesiredStateInput() = default;

template <typename T>
void DesiredStateInput<T>::SetModelInstanceDesiredStates(
    ModelInstanceIndex model_instance, VectorX<T> qd, VectorX<T> vd) {
  DRAKE_DEMAND(model_instance < num_model_instances());
  positions_[model_instance] = std::move(qd);
  velocities_[model_instance] = std::move(vd);
}

}  // namespace internal
}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::multibody::internal::DesiredStateInput);
