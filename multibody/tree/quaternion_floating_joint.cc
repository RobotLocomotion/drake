#include "drake/multibody/tree/quaternion_floating_joint.h"

#include <memory>
#include <stdexcept>

#include "drake/multibody/tree/multibody_tree.h"

namespace drake {
namespace multibody {

template <typename T>
QuaternionFloatingJoint<T>::~QuaternionFloatingJoint() = default;

template <typename T>
template <typename ToScalar>
std::unique_ptr<Joint<ToScalar>>
QuaternionFloatingJoint<T>::TemplatedDoCloneToScalar(
    const internal::MultibodyTree<ToScalar>& tree_clone) const {
  const Frame<ToScalar>& frame_on_parent_body_clone =
      tree_clone.get_variant(this->frame_on_parent());
  const Frame<ToScalar>& frame_on_child_body_clone =
      tree_clone.get_variant(this->frame_on_child());

  // Make the Joint<T> clone.
  auto joint_clone = std::make_unique<QuaternionFloatingJoint<ToScalar>>(
      this->name(), frame_on_parent_body_clone, frame_on_child_body_clone,
      this->default_angular_damping(), this->default_translational_damping());
  joint_clone->set_position_limits(this->position_lower_limits(),
                                   this->position_upper_limits());
  joint_clone->set_velocity_limits(this->velocity_lower_limits(),
                                   this->velocity_upper_limits());
  joint_clone->set_acceleration_limits(this->acceleration_lower_limits(),
                                       this->acceleration_upper_limits());
  joint_clone->set_default_positions(this->default_positions());

  return joint_clone;
}

template <typename T>
std::unique_ptr<Joint<double>> QuaternionFloatingJoint<T>::DoCloneToScalar(
    const internal::MultibodyTree<double>& tree_clone) const {
  return TemplatedDoCloneToScalar(tree_clone);
}

template <typename T>
std::unique_ptr<Joint<AutoDiffXd>> QuaternionFloatingJoint<T>::DoCloneToScalar(
    const internal::MultibodyTree<AutoDiffXd>& tree_clone) const {
  return TemplatedDoCloneToScalar(tree_clone);
}

template <typename T>
std::unique_ptr<Joint<symbolic::Expression>>
QuaternionFloatingJoint<T>::DoCloneToScalar(
    const internal::MultibodyTree<symbolic::Expression>& tree_clone) const {
  return TemplatedDoCloneToScalar(tree_clone);
}

// N.B. Due to esoteric linking errors on Mac (see #9345) involving
// `MobilizerImpl`, we must place this implementation in the source file, not
// in the header file.
template <typename T>
std::unique_ptr<typename Joint<T>::BluePrint>
QuaternionFloatingJoint<T>::MakeImplementationBlueprint(
    const internal::SpanningForest::Mobod& mobod) const {
  auto blue_print = std::make_unique<typename Joint<T>::BluePrint>();
  const auto [inboard_frame, outboard_frame] =
      this->tree_frames(mobod.is_reversed());
  // TODO(sherm1) The mobilizer needs to be reversed, not just the frames.
  auto quaternion_floating_mobilizer =
      std::make_unique<internal::QuaternionFloatingMobilizer<T>>(
          mobod, *inboard_frame, *outboard_frame);
  quaternion_floating_mobilizer->set_default_position(
      this->default_positions());
  blue_print->mobilizer = std::move(quaternion_floating_mobilizer);
  return blue_print;
}

template <typename T>
void QuaternionFloatingJoint<T>::DoAddInOneForce(const systems::Context<T>&,
                                                 int, const T&,
                                                 MultibodyForces<T>*) const {
  throw std::logic_error(
      "QuaternionFloating joints do not allow applying forces to individual "
      "degrees of freedom.");
}

template <typename T>
void QuaternionFloatingJoint<T>::DoAddInDamping(
    const systems::Context<T>& context, MultibodyForces<T>* forces) const {
  Eigen::Ref<VectorX<T>> t_BMo_F =
      get_mobilizer().get_mutable_generalized_forces_from_array(
          &forces->mutable_generalized_forces());
  const Vector3<T>& w_FM = get_angular_velocity(context);
  const Vector3<T>& v_FM = get_translational_velocity(context);
  const T& angular_damping = this->GetDampingVector(context)[0];
  const T& translational_damping = this->GetDampingVector(context)[3];
  t_BMo_F.template head<3>() -= angular_damping * w_FM;
  t_BMo_F.template tail<3>() -= translational_damping * v_FM;
}

namespace internal {

template <typename T>
const QuaternionFloatingJoint<T>*
DynamicCastJoint<QuaternionFloatingJoint>::cast(const Joint<T>* element) {
  return dynamic_cast<const QuaternionFloatingJoint<T>*>(element);
}

// clang-format off
DRAKE_DEFINE_FUNCTION_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS((
    &DynamicCastJoint<QuaternionFloatingJoint>::template cast<T>
));
// clang-format on

}  // namespace internal
}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::multibody::QuaternionFloatingJoint);
