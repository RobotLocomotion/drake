#include "drake/multibody/fem/fem_state.h"

namespace drake {
namespace multibody {
namespace fem {

template <typename T>
FemState<T>::FemState(const internal::FemStateSystem<T>* system)
    : system_(system) {
  DRAKE_DEMAND(system != nullptr);
  owned_context_ = system_->CreateDefaultContext();
}

template <typename T>
FemState<T>::FemState(const internal::FemStateSystem<T>* system,
                      const systems::Context<T>* context)
    : system_(system), context_(context) {
  DRAKE_DEMAND(system != nullptr);
  DRAKE_DEMAND(context != nullptr);
  system->ValidateContext(*context);
}

template <typename T>
const VectorX<T>& FemState<T>::GetPositions() const {
  return get_context()
      .get_discrete_state(system_->fem_position_index())
      .value();
}

template <typename T>
const VectorX<T>& FemState<T>::GetVelocities() const {
  return get_context()
      .get_discrete_state(system_->fem_velocity_index())
      .value();
}

template <typename T>
const VectorX<T>& FemState<T>::GetAccelerations() const {
  return get_context()
      .get_discrete_state(system_->fem_acceleration_index())
      .value();
}

template <typename T>
void FemState<T>::SetPositions(const Eigen::Ref<const VectorX<T>>& q) {
  get_mutable_context().SetDiscreteState(system_->fem_position_index(), q);
}

template <typename T>
void FemState<T>::SetVelocities(const Eigen::Ref<const VectorX<T>>& v) {
  get_mutable_context().SetDiscreteState(system_->fem_velocity_index(), v);
}

template <typename T>
void FemState<T>::SetAccelerations(const Eigen::Ref<const VectorX<T>>& a) {
  get_mutable_context().SetDiscreteState(system_->fem_acceleration_index(), a);
}

}  // namespace fem
}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::multibody::fem::FemState);
