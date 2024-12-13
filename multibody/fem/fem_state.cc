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
const VectorX<T>& FemState<T>::GetPreviousStepPositions() const {
  return get_context()
      .get_discrete_state(system_->fem_previous_step_position_index())
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
void FemState<T>::SetTimeStepPositions(const Eigen::Ref<const VectorX<T>>& q0) {
  get_mutable_context().SetDiscreteState(
      system_->fem_previous_step_position_index(), q0);
}

template <typename T>
void FemState<T>::SetVelocities(const Eigen::Ref<const VectorX<T>>& v) {
  get_mutable_context().SetDiscreteState(system_->fem_velocity_index(), v);
}

template <typename T>
void FemState<T>::SetAccelerations(const Eigen::Ref<const VectorX<T>>& a) {
  get_mutable_context().SetDiscreteState(system_->fem_acceleration_index(), a);
}

template <typename T>
void FemState<T>::CopyFrom(const FemState<T>& other) {
  DRAKE_THROW_UNLESS(num_dofs() == other.num_dofs());
  if (owned_context_ == nullptr)
    throw std::runtime_error("Trying to mutate a shared FemState.");
  owned_context_->SetTimeStateAndParametersFrom(other.get_context());
}

template <typename T>
std::unique_ptr<FemState<T>> FemState<T>::Clone() const {
  if (owned_context_ != nullptr) {
    auto clone = std::make_unique<FemState<T>>(this->system_);
    clone->owned_context_->SetTimeStateAndParametersFrom(*this->owned_context_);
    return clone;
  }
  DRAKE_DEMAND(context_ != nullptr);
  // Note that this creates a "shared context" clone. See the class
  // documentation for cautionary notes.
  return std::make_unique<FemState<T>>(this->system_, this->context_);
}

}  // namespace fem
}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::multibody::fem::FemState);
