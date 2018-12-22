#include "drake/multibody/tree/revolute_joint.h"

#include <memory>
#include <stdexcept>

#include "drake/common/autodiff.h"
#include "drake/multibody/tree/multibody_tree.h"

namespace drake {
namespace multibody {

template <typename T>
template <typename ToScalar>
std::unique_ptr<Joint<ToScalar>>
RevoluteJoint<T>::TemplatedDoCloneToScalar(
    const internal::MultibodyTree<ToScalar>& tree_clone) const {
  const Frame<ToScalar>& frame_on_parent_body_clone =
      tree_clone.get_variant(this->frame_on_parent());
  const Frame<ToScalar>& frame_on_child_body_clone =
      tree_clone.get_variant(this->frame_on_child());

  // Make the Joint<T> clone.
  auto joint_clone = std::make_unique<RevoluteJoint<ToScalar>>(
      this->name(),
      frame_on_parent_body_clone, frame_on_child_body_clone,
      this->revolute_axis(),
      this->lower_limit(), this->upper_limit(), this->damping());

  return std::move(joint_clone);
}

template <typename T>
std::unique_ptr<Joint<double>> RevoluteJoint<T>::DoCloneToScalar(
    const internal::MultibodyTree<double>& tree_clone) const {
  return TemplatedDoCloneToScalar(tree_clone);
}

template <typename T>
std::unique_ptr<Joint<AutoDiffXd>> RevoluteJoint<T>::DoCloneToScalar(
    const internal::MultibodyTree<AutoDiffXd>& tree_clone) const {
  return TemplatedDoCloneToScalar(tree_clone);
}

// N.B. Due to esoteric linking errors on Mac (see #9345) involving
// `MobilizerImpl`, we must place this implementation in the source file, not
// in the header file.
template <typename T>
std::unique_ptr<typename Joint<T>::BluePrint>
RevoluteJoint<T>::MakeImplementationBlueprint() const {
  auto blue_print = std::make_unique<typename Joint<T>::BluePrint>();
  blue_print->mobilizers_.push_back(
      std::make_unique<internal::RevoluteMobilizer<T>>(
          this->frame_on_parent(), this->frame_on_child(), axis_));
  return std::move(blue_print);
}

// Explicitly instantiates on the most common scalar types.
template class RevoluteJoint<double>;
template class RevoluteJoint<AutoDiffXd>;

}  // namespace multibody
}  // namespace drake
