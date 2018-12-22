#include "drake/multibody/tree/prismatic_joint.h"

#include <memory>

#include "drake/common/autodiff.h"
#include "drake/multibody/tree/multibody_tree.h"

namespace drake {
namespace multibody {

template <typename T>
template <typename ToScalar>
std::unique_ptr<Joint<ToScalar>>
PrismaticJoint<T>::TemplatedDoCloneToScalar(
    const internal::MultibodyTree<ToScalar>& tree_clone) const {
  const Frame<ToScalar>& frame_on_parent_body_clone =
      tree_clone.get_variant(this->frame_on_parent());
  const Frame<ToScalar>& frame_on_child_body_clone =
      tree_clone.get_variant(this->frame_on_child());

  // Make the Joint<T> clone.
  auto joint_clone = std::make_unique<PrismaticJoint<ToScalar>>(
      this->name(),
      frame_on_parent_body_clone, frame_on_child_body_clone,
      this->translation_axis(),
      this->lower_limit(), this->upper_limit(), this->damping());

  return joint_clone;
}

template <typename T>
std::unique_ptr<Joint<double>> PrismaticJoint<T>::DoCloneToScalar(
    const internal::MultibodyTree<double>& tree_clone) const {
  return TemplatedDoCloneToScalar(tree_clone);
}

template <typename T>
std::unique_ptr<Joint<AutoDiffXd>> PrismaticJoint<T>::DoCloneToScalar(
    const internal::MultibodyTree<AutoDiffXd>& tree_clone) const {
  return TemplatedDoCloneToScalar(tree_clone);
}

// Explicitly instantiates on the most common scalar types.
template class PrismaticJoint<double>;
template class PrismaticJoint<AutoDiffXd>;

}  // namespace multibody
}  // namespace drake
