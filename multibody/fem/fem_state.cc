#include "drake/multibody/fem/fem_state.h"

namespace drake {
namespace multibody {
namespace fem {

template <typename T>
FemState<T>::FemState(const internal::FemStateInfo<T>& data_info)
    : info_(data_info) {
  context_ = info_.system->CreateDefaultContext();
}

template <typename T>
const VectorX<T>& FemState<T>::GetPositions() const {
  return context_->get_discrete_state(info_.fem_position_index).value();
}

template <typename T>
const VectorX<T>& FemState<T>::GetVelocities() const {
  return context_->get_discrete_state(info_.fem_velocity_index).value();
}

template <typename T>
const VectorX<T>& FemState<T>::GetAccelerations() const {
  return context_->get_discrete_state(info_.fem_acceleration_index).value();
}

template <typename T>
void FemState<T>::SetPositions(const Eigen::Ref<const VectorX<T>>& q) {
  context_->SetDiscreteState(info_.fem_position_index, q);
}

template <typename T>
void FemState<T>::SetVelocities(const Eigen::Ref<const VectorX<T>>& v) {
  context_->SetDiscreteState(info_.fem_velocity_index, v);
}

template <typename T>
void FemState<T>::SetAccelerations(const Eigen::Ref<const VectorX<T>>& a) {
  context_->SetDiscreteState(info_.fem_acceleration_index, a);
}

}  // namespace fem
}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::multibody::fem::FemState);
