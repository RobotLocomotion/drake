#include "drake/multibody/tree/joint.h"

namespace drake {
namespace multibody {

template <typename T>
Joint<T>::~Joint() = default;

template <typename T>
bool Joint<T>::can_rotate() const {
  DRAKE_DEMAND(has_mobilizer());
  return mobilizer_->can_rotate();
}

template <typename T>
bool Joint<T>::can_translate() const {
  DRAKE_DEMAND(has_mobilizer());
  return mobilizer_->can_translate();
}

template <typename T>
void Joint<T>::set_default_positions(const VectorX<double>& default_positions) {
  if (default_positions.size() != num_positions()) {
    std::string model_instance_name;
    if (this->has_parent_tree()) {
      model_instance_name =
          this->get_parent_tree().GetModelInstanceName(this->model_instance()) +
          "::";
    }
    throw std::runtime_error(fmt::format(
        "{}: The number of positions in the input ({}) does not match the "
        "number of positions of the joint '{}{}' ({}).",
        __func__, default_positions.size(), model_instance_name, name(),
        num_positions()));
  }
  default_positions_ = default_positions;
  do_set_default_positions(default_positions);
}

template <typename T>
void Joint<T>::SetPositions(
    systems::Context<T>* context,
    const Eigen::Ref<const VectorX<T>>& positions) const {
  DRAKE_THROW_UNLESS(context != nullptr);
  DRAKE_THROW_UNLESS(positions.size() == num_positions());
  DRAKE_THROW_UNLESS(this->has_parent_tree());
  this->get_parent_tree().ThrowIfNotFinalized("Joint::SetPositions");
  DRAKE_DEMAND(has_mobilizer());
  const Eigen::VectorBlock<VectorX<T>> all_q =
      this->get_parent_tree().GetMutablePositions(&*context);
  mobilizer_->get_mutable_positions_from_array(&all_q) = positions;
}

template <typename T>
Eigen::Ref<const VectorX<T>> Joint<T>::GetPositions(
    const systems::Context<T>& context) const {
  DRAKE_THROW_UNLESS(this->has_parent_tree());
  this->get_parent_tree().ThrowIfNotFinalized("Joint::GetPositions");
  DRAKE_DEMAND(has_mobilizer());
  const Eigen::VectorBlock<const VectorX<T>> all_q =
      this->get_parent_tree().get_positions(context);
  return mobilizer_->get_positions_from_array(all_q);
}

template <typename T>
void Joint<T>::SetVelocities(
    systems::Context<T>* context,
    const Eigen::Ref<const VectorX<T>>& velocities) const {
  DRAKE_THROW_UNLESS(context != nullptr);
  DRAKE_THROW_UNLESS(velocities.size() == num_velocities());
  DRAKE_THROW_UNLESS(this->has_parent_tree());
  this->get_parent_tree().ThrowIfNotFinalized("Joint::SetVelocities");
  DRAKE_DEMAND(has_mobilizer());
  const Eigen::VectorBlock<VectorX<T>> all_v =
      this->get_parent_tree().GetMutableVelocities(&*context);
  mobilizer_->get_mutable_velocities_from_array(&all_v) = velocities;
}

template <typename T>
Eigen::Ref<const VectorX<T>> Joint<T>::GetVelocities(
    const systems::Context<T>& context) const {
  DRAKE_THROW_UNLESS(this->has_parent_tree());
  this->get_parent_tree().ThrowIfNotFinalized("Joint::GetVelocities");
  DRAKE_DEMAND(has_mobilizer());
  const Eigen::VectorBlock<const VectorX<T>> all_v =
      this->get_parent_tree().get_velocities(context);
  return mobilizer_->get_velocities_from_array(all_v);
}

template <typename T>
std::unique_ptr<internal::Mobilizer<T>> Joint<T>::Build(
    const internal::SpanningForest::Mobod& mobod,
    internal::MultibodyTree<T>* tree) {
  DRAKE_DEMAND(tree != nullptr);
  std::unique_ptr<internal::Mobilizer<T>> owned_mobilizer =
      MakeMobilizerForJoint(mobod, tree);
  mobilizer_ = owned_mobilizer.get();
  return owned_mobilizer;
}

template <typename T>
std::unique_ptr<Joint<T>> Joint<T>::ShallowClone() const {
  std::unique_ptr<Joint<T>> result = DoShallowClone();
  DRAKE_THROW_UNLESS(result != nullptr);
  // N.B. We can't call set_default_damping_vector because it segfaults when
  // there's no parent tree.
  result->damping_ = this->damping_;
  result->set_position_limits(position_lower_limits(), position_upper_limits());
  result->set_velocity_limits(velocity_lower_limits(), velocity_upper_limits());
  result->set_acceleration_limits(acceleration_lower_limits(),
                                  acceleration_upper_limits());
  result->set_default_positions(default_positions());
  return result;
}

template <typename T>
std::unique_ptr<Joint<T>> Joint<T>::DoShallowClone() const {
  throw std::logic_error(fmt::format(
      "The {} joint failed to override DoShallowClone()", type_name()));
}

template <typename T>
std::string Joint<T>::MakeUniqueOffsetFrameName(
    const Frame<T>& parent_frame, const std::string& suffix) const {
  DRAKE_DEMAND(this->has_parent_tree());
  const internal::MultibodyTree<T>& tree = this->get_parent_tree();
  std::string new_name =
      fmt::format("{}_{}_{}", this->name(), parent_frame.name(), suffix);
  while (tree.HasFrameNamed(new_name, this->model_instance())) {
    new_name = "_" + new_name;
  }
  return new_name;
}

template <typename T>
void Joint<T>::SetSpatialVelocityImpl(systems::Context<T>* context,
                                      const SpatialVelocity<T>& V_FM,
                                      const char* func) const {
  DRAKE_THROW_UNLESS(context != nullptr);
  DRAKE_THROW_UNLESS(this->has_parent_tree());
  this->get_parent_tree().ThrowIfNotFinalized("Joint::SetSpatialVelocity");
  DRAKE_DEMAND(has_mobilizer());
  if (!mobilizer_->SetSpatialVelocity(*context, V_FM,
                                      &context->get_mutable_state())) {
    throw std::logic_error(
        fmt::format("{}(): {} joint does not implement this function "
                    "(joint '{}')",
                    func, type_name(), name()));
  }
}

template <typename T>
SpatialVelocity<T> Joint<T>::GetSpatialVelocity(
    const systems::Context<T>& context) const {
  DRAKE_THROW_UNLESS(this->has_parent_tree());
  this->get_parent_tree().ThrowIfNotFinalized("Joint::GetSpatialVelocity");
  DRAKE_DEMAND(has_mobilizer());
  return mobilizer_->GetSpatialVelocity(context);
}

template <typename T>
void Joint<T>::SetPosePairImpl(systems::Context<T>* context,
                               const Quaternion<T>& q_FM,
                               const Vector3<T>& p_FM, const char* func) const {
  DRAKE_THROW_UNLESS(context != nullptr);
  DRAKE_THROW_UNLESS(this->has_parent_tree());
  this->get_parent_tree().ThrowIfNotFinalized("Joint::SetPosePair");
  DRAKE_DEMAND(has_mobilizer());
  if (!mobilizer_->SetPosePair(*context, q_FM, p_FM,
                               &context->get_mutable_state())) {
    throw std::logic_error(
        fmt::format("{}(): {} joint does not implement this function "
                    "(joint '{}')",
                    func, type_name(), name()));
  }
}

template <typename T>
std::pair<Eigen::Quaternion<T>, Vector3<T>> Joint<T>::GetPosePair(
    const systems::Context<T>& context) const {
  DRAKE_THROW_UNLESS(this->has_parent_tree());
  this->get_parent_tree().ThrowIfNotFinalized("Joint::GetPosePair");
  DRAKE_DEMAND(has_mobilizer());
  return mobilizer_->GetPosePair(context);
}

}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class drake::multibody::Joint);
