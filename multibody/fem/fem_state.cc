#include "drake/multibody/fem/fem_state.h"

namespace drake {
namespace multibody {
namespace fem {

template <typename T>
FemState<T>::FemState(const internal::FemStateManager<T>* manager)
    : manager_(manager) {
  DRAKE_DEMAND(manager != nullptr);
  owned_context_ = manager_->CreateDefaultContext();
}

template <typename T>
FemState<T>::FemState(const internal::FemStateManager<T>* manager,
                      const systems::Context<T>* context)
    : manager_(manager), context_(context) {
  DRAKE_DEMAND(manager != nullptr);
  DRAKE_DEMAND(context != nullptr);
  manager->ValidateContext(*context);
}

template <typename T>
const VectorX<T>& FemState<T>::GetPositions() const {
  return get_context()
      .get_discrete_state(manager_->fem_position_index())
      .value();
}

template <typename T>
const VectorX<T>& FemState<T>::GetVelocities() const {
  return get_context()
      .get_discrete_state(manager_->fem_velocity_index())
      .value();
}

template <typename T>
const VectorX<T>& FemState<T>::GetAccelerations() const {
  return get_context()
      .get_discrete_state(manager_->fem_acceleration_index())
      .value();
}

template <typename T>
void FemState<T>::SetPositions(const Eigen::Ref<const VectorX<T>>& q) {
  get_mutable_context().SetDiscreteState(manager_->fem_position_index(), q);
}

template <typename T>
void FemState<T>::SetVelocities(const Eigen::Ref<const VectorX<T>>& v) {
  get_mutable_context().SetDiscreteState(manager_->fem_velocity_index(), v);
}

template <typename T>
void FemState<T>::SetAccelerations(const Eigen::Ref<const VectorX<T>>& a) {
  get_mutable_context().SetDiscreteState(manager_->fem_acceleration_index(), a);
}

}  // namespace fem
}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::multibody::fem::FemState);
