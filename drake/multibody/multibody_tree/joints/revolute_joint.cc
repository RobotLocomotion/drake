#include "drake/multibody/multibody_tree/joints/revolute_joint.h"

#include <memory>
#include <stdexcept>

#include "drake/common/eigen_autodiff_types.h"
#include "drake/multibody/multibody_tree/multibody_tree.h"

namespace drake {
namespace multibody {

template <typename T>
template <typename ToScalar>
std::unique_ptr<Joint<ToScalar>>
RevoluteJoint<T>::TemplatedDoCloneToScalar(
    const MultibodyTree<ToScalar>& tree_clone) const {
  const RigidBody<ToScalar>& inboard_body_clone =
      tree_clone.get_variant(this->get_parent_body());
  const RigidBody<ToScalar>& outboard_body_clone =
      tree_clone.get_variant(this->get_child_body());

  // Make the Joint<T> clone.
  auto joint_clone = std::make_unique<RevoluteJoint<ToScalar>>(
      this->get_name(),
      inboard_body_clone, this->get_frame_on_parent_pose(),
      outboard_body_clone, this->get_frame_on_child_pose(),
      this->get_revolute_axis());

  return std::move(joint_clone);
}

template <typename T>
std::unique_ptr<Joint<double>> RevoluteJoint<T>::DoCloneToScalar(
    const MultibodyTree<double>& tree_clone) const {
  return TemplatedDoCloneToScalar(tree_clone);
}

template <typename T>
std::unique_ptr<Joint<AutoDiffXd>> RevoluteJoint<T>::DoCloneToScalar(
    const MultibodyTree<AutoDiffXd>& tree_clone) const {
  return TemplatedDoCloneToScalar(tree_clone);
}

// Explicitly instantiates on the most common scalar types.
template class RevoluteJoint<double>;
template class RevoluteJoint<AutoDiffXd>;

}  // namespace multibody
}  // namespace drake
