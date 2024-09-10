#include "drake/multibody/tree/joint.h"

namespace drake {
namespace multibody {

template <typename T>
Joint<T>::~Joint() = default;

template <typename T>
bool Joint<T>::can_rotate() const {
  DRAKE_DEMAND(this->get_implementation().has_mobilizer());
  return get_implementation().mobilizer->can_rotate();
}

template <typename T>
bool Joint<T>::can_translate() const {
  DRAKE_DEMAND(this->get_implementation().has_mobilizer());
  return get_implementation().mobilizer->can_translate();
}

template <typename T>
void Joint<T>::set_default_positions(const VectorX<double>& default_positions) {
  DRAKE_THROW_UNLESS(default_positions.size() == num_positions());
  default_positions_ = default_positions;
  do_set_default_positions(default_positions);
}

template <typename T>
void Joint<T>::SetPositions(
    systems::Context<T>* context,
    const Eigen::Ref<const VectorX<T>>& positions) const {
  DRAKE_THROW_UNLESS(context != nullptr);
  DRAKE_THROW_UNLESS(positions.size() == num_positions());
  this->get_parent_tree().ThrowIfNotFinalized("Joint::SetPositions");
  DRAKE_DEMAND(get_implementation().has_mobilizer());
  const Eigen::VectorBlock<VectorX<T>> all_q =
      this->get_parent_tree().GetMutablePositions(&*context);
  get_implementation().mobilizer->get_mutable_positions_from_array(&all_q) =
      positions;
}

template <typename T>
Eigen::Ref<const VectorX<T>> Joint<T>::GetPositions(
    const systems::Context<T>& context) const {
  this->get_parent_tree().ThrowIfNotFinalized("Joint::GetPositions");
  DRAKE_DEMAND(get_implementation().has_mobilizer());
  const Eigen::VectorBlock<const VectorX<T>> all_q =
      this->get_parent_tree().get_positions(context);
  return get_implementation().mobilizer->get_positions_from_array(all_q);
}

template <typename T>
void Joint<T>::SetVelocities(
    systems::Context<T>* context,
    const Eigen::Ref<const VectorX<T>>& velocities) const {
  DRAKE_THROW_UNLESS(context != nullptr);
  DRAKE_THROW_UNLESS(velocities.size() == num_velocities());
  this->get_parent_tree().ThrowIfNotFinalized("Joint::SetVelocities");
  DRAKE_DEMAND(get_implementation().has_mobilizer());
  const Eigen::VectorBlock<VectorX<T>> all_v =
      this->get_parent_tree().GetMutableVelocities(&*context);
  get_implementation().mobilizer->get_mutable_velocities_from_array(&all_v) =
      velocities;
}

template <typename T>
Eigen::Ref<const VectorX<T>> Joint<T>::GetVelocities(
    const systems::Context<T>& context) const {
  this->get_parent_tree().ThrowIfNotFinalized("Joint::GetVelocities");
  DRAKE_DEMAND(get_implementation().has_mobilizer());
  const Eigen::VectorBlock<const VectorX<T>> all_v =
      this->get_parent_tree().get_velocities(context);
  return get_implementation().mobilizer->get_velocities_from_array(all_v);
}

template <typename T>
void Joint<T>::SetSpatialVelocity(systems::Context<T>* context,
                                  const SpatialVelocity<T>& V_FM) const {
  DRAKE_THROW_UNLESS(context != nullptr);
  this->get_parent_tree().ThrowIfNotFinalized("Joint::SetSpatialVelocity");
  DRAKE_DEMAND(get_implementation().has_mobilizer());
  get_implementation().mobilizer->SetSpatialVelocity(
      *context, V_FM, &context->get_mutable_state());
}

template <typename T>
SpatialVelocity<T> Joint<T>::GetSpatialVelocity(
    const systems::Context<T>& context) const {
  this->get_parent_tree().ThrowIfNotFinalized("Joint::GetSpatialVelocity");
  DRAKE_DEMAND(get_implementation().has_mobilizer());
  return get_implementation().mobilizer->GetSpatialVelocity(context);
}

template <typename T>
void Joint<T>::SetPosePair(systems::Context<T>* context,
                           const Quaternion<T>& q_FM,
                           const Vector3<T>& p_FM) const {
  DRAKE_THROW_UNLESS(context != nullptr);
  this->get_parent_tree().ThrowIfNotFinalized("Joint::SetPosePair");
  DRAKE_DEMAND(get_implementation().has_mobilizer());
  get_implementation().mobilizer->SetPosePair(*context, q_FM, p_FM,
                                              &context->get_mutable_state());
}

template <typename T>
std::pair<Eigen::Quaternion<T>, Vector3<T>> Joint<T>::GetPosePair(
    const systems::Context<T>& context) const {
  this->get_parent_tree().ThrowIfNotFinalized("Joint::GetPosePair");
  DRAKE_DEMAND(get_implementation().has_mobilizer());
  return get_implementation().mobilizer->GetPosePair(context);
}

}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class drake::multibody::Joint);
