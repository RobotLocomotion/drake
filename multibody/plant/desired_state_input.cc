#include "drake/multibody/plant/desired_state_input.h"

#include <utility>

#include "drake/common/default_scalars.h"

namespace drake {
namespace multibody {
namespace internal {

template <typename T>
DesiredStateInput<T>::DesiredStateInput(int num_model_instances) {
  DRAKE_DEMAND(num_model_instances > 0);
  positions_.resize(num_model_instances);
  velocities_.resize(num_model_instances);
}

template <typename T>
DesiredStateInput<T>::~DesiredStateInput() = default;

template <typename T>
void DesiredStateInput<T>::SetModelInstanceDesiredStates(
    ModelInstanceIndex model_instance, const Eigen::Ref<const VectorX<T>>& qd,
    const Eigen::Ref<const VectorX<T>>& vd) {
  DRAKE_DEMAND(model_instance < num_model_instances());
  DRAKE_DEMAND(qd.size() == vd.size());
  positions_[model_instance] = qd;
  velocities_[model_instance] = vd;
}

template <typename T>
void DesiredStateInput<T>::ClearModelInstanceDesiredStates(
    ModelInstanceIndex model_instance) {
  DRAKE_DEMAND(model_instance < num_model_instances());
  positions_[model_instance] = std::nullopt;
  velocities_[model_instance] = std::nullopt;
}

}  // namespace internal
}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::multibody::internal::DesiredStateInput);
