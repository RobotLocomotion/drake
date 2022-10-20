#include "drake/multibody/tree/weld_joint.h"

#include <memory>

#include "drake/multibody/tree/multibody_tree.h"
#include "drake/multibody/tree/multibody_tree-inl.h"

namespace drake {
namespace multibody {

template <typename T>
const std::string& WeldJoint<T>::type_name() const {
  static const never_destroyed<std::string> name{kTypeName};
  return name.access();
}

template <typename T>
template <typename ToScalar>
std::unique_ptr<Joint<ToScalar>> WeldJoint<T>::TemplatedDoCloneToScalar(
    const Frame<ToScalar>& frame_on_parent_body_clone,
    const Frame<ToScalar>& frame_on_child_body_clone) const {
  // Make the Joint<T> clone.
  auto joint_clone = std::make_unique<WeldJoint<ToScalar>>(
      this->name(), frame_on_parent_body_clone, frame_on_child_body_clone,
      X_FM());

  return joint_clone;
}

template <typename T>
template <typename ToScalar>
std::unique_ptr<Joint<ToScalar>>
WeldJoint<T>::TemplatedDoCloneToScalar(
    const internal::MultibodyTree<ToScalar>& tree_clone) const {
  const Frame<ToScalar>& frame_on_parent_body_clone =
      tree_clone.get_variant(this->frame_on_parent());
  const Frame<ToScalar>& frame_on_child_body_clone =
      tree_clone.get_variant(this->frame_on_child());

  return TemplatedDoCloneToScalar(frame_on_parent_body_clone,
                                  frame_on_child_body_clone);
}

template <typename T>
std::unique_ptr<Joint<double>> WeldJoint<T>::DoCloneToScalar(
    const internal::MultibodyTree<double>& tree_clone) const {
  return TemplatedDoCloneToScalar(tree_clone);
}

template <typename T>
std::unique_ptr<Joint<AutoDiffXd>> WeldJoint<T>::DoCloneToScalar(
    const internal::MultibodyTree<AutoDiffXd>& tree_clone) const {
  return TemplatedDoCloneToScalar(tree_clone);
}

template <typename T>
std::unique_ptr<Joint<symbolic::Expression>> WeldJoint<T>::DoCloneToScalar(
    const internal::MultibodyTree<symbolic::Expression>& tree_clone) const {
  return TemplatedDoCloneToScalar(tree_clone);
}

template <typename T>
const Joint<T>& WeldJoint<T>::DoCloneTo(
    internal::MultibodyTree<T>* tree, const Frame<T>& dest_frame_on_parent,
    const Frame<T>& dest_frame_on_child) const {
  return tree->AddJoint(
      TemplatedDoCloneToScalar(dest_frame_on_parent, dest_frame_on_child));
}

// N.B. Due to esoteric linking errors on Mac (see #9345) involving
// `MobilizerImpl`, we must place this implementation in the source file, not
// in the header file.
template <typename T>
std::unique_ptr<typename Joint<T>::BluePrint>
WeldJoint<T>::MakeImplementationBlueprint() const {
  auto blue_print = std::make_unique<typename Joint<T>::BluePrint>();
  blue_print->mobilizers_.push_back(
      std::make_unique<internal::WeldMobilizer<T>>(
          this->frame_on_parent(), this->frame_on_child(), X_FM_));
  return blue_print;
}

}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::multibody::WeldJoint)
