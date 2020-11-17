#include "drake/multibody/tree/planar_joint.h"

#include <memory>

#include "drake/multibody/tree/multibody_tree.h"

namespace drake {
namespace multibody {

template <typename T>
template <typename ToScalar>
std::unique_ptr<Joint<ToScalar>> PlanarJoint<T>::TemplatedDoCloneToScalar(
    const internal::MultibodyTree<ToScalar>& tree_clone) const {
  const Frame<ToScalar>& frame_on_parent_body_clone =
      tree_clone.get_variant(this->frame_on_parent());
  const Frame<ToScalar>& frame_on_child_body_clone =
      tree_clone.get_variant(this->frame_on_child());

  // Make the Joint<T> clone.
  auto joint_clone = std::make_unique<PlanarJoint<ToScalar>>(
      this->name(), frame_on_parent_body_clone, frame_on_child_body_clone,
      this->damping());
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
std::unique_ptr<Joint<double>> PlanarJoint<T>::DoCloneToScalar(
    const internal::MultibodyTree<double>& tree_clone) const {
  return TemplatedDoCloneToScalar(tree_clone);
}

template <typename T>
std::unique_ptr<Joint<AutoDiffXd>> PlanarJoint<T>::DoCloneToScalar(
    const internal::MultibodyTree<AutoDiffXd>& tree_clone) const {
  return TemplatedDoCloneToScalar(tree_clone);
}

template <typename T>
std::unique_ptr<Joint<symbolic::Expression>> PlanarJoint<T>::DoCloneToScalar(
    const internal::MultibodyTree<symbolic::Expression>& tree_clone) const {
  return TemplatedDoCloneToScalar(tree_clone);
}

// N.B. Due to esoteric linking errors on Mac (see #9345) involving
// `MobilizerImpl`, we must place this implementation in the source file, not
// in the header file.
template <typename T>
std::unique_ptr<typename Joint<T>::BluePrint>
PlanarJoint<T>::MakeImplementationBlueprint() const {
  auto blue_print = std::make_unique<typename Joint<T>::BluePrint>();
  auto planar_mobilizer = std::make_unique<internal::PlanarMobilizer<T>>(
      this->frame_on_parent(), this->frame_on_child());
  planar_mobilizer->set_default_position(this->default_positions());
  blue_print->mobilizers_.push_back(std::move(planar_mobilizer));
  return blue_print;
}

}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::multibody::PlanarJoint)
