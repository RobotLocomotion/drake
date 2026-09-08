#include "drake/multibody/tree/weld_joint.h"

#include <memory>

#include "drake/multibody/tree/multibody_tree-inl.h"

namespace drake {
namespace multibody {

template <typename T>
WeldJoint<T>::~WeldJoint() = default;

template <typename T>
const std::string& WeldJoint<T>::type_name() const {
  static const never_destroyed<std::string> name{kTypeName};
  return name.access();
}

template <typename T>
template <typename ToScalar>
std::unique_ptr<Joint<ToScalar>> WeldJoint<T>::TemplatedDoCloneToScalar(
    const internal::MultibodyTree<ToScalar>& tree_clone) const {
  const Frame<ToScalar>& frame_on_parent_body_clone =
      tree_clone.get_variant(this->frame_on_parent());
  const Frame<ToScalar>& frame_on_child_body_clone =
      tree_clone.get_variant(this->frame_on_child());

  // Make the Joint<T> clone.
  auto joint_clone = std::make_unique<WeldJoint<ToScalar>>(
      this->name(), frame_on_parent_body_clone, frame_on_child_body_clone,
      X_FM());

  return joint_clone;
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
std::unique_ptr<Joint<T>> WeldJoint<T>::DoShallowClone() const {
  return std::make_unique<WeldJoint<T>>(this->name(), this->frame_on_parent(),
                                        this->frame_on_child(), X_FM());
}

/* For a weld joint, we are given Jp on parent P, Jc on child C, and a fixed
X_JpJc. For optimal performance, the underlying mobilizer expects coincident
frames F (on the inboard body) and M (on the outboard body) so that we don't
have to apply X_JpJc repeatedly at run time. We only need to move one of the
frames, and because of the way we report reaction forces (at Jc) it is best
to choose M=Jc and move F if necessary by creating a new offset frame from Jp
that is coincident with Jc (and M):
    X_JpF = X_JpJc
This yields X_FM = X_FJc = (X_JpF)⁻¹ * X_JpJc = Identity as required.
If the mobilizer's inboard/outboard direction is reversed from the joint's
parent/child direction, we instead choose M=Jp and an offset from F
    X_JcF = (X_JpJc)⁻¹
so that X_FM = X_FJp = (X_JcF)⁻¹ (X_JpJc)⁻¹ = Identity.

If X_JpJc is already Identity we just use the original frames for F and M. */
template <typename T>
std::unique_ptr<internal::Mobilizer<T>> WeldJoint<T>::MakeMobilizerForJoint(
    const internal::SpanningForest::Mobod& mobod,
    internal::MultibodyTree<T>* tree) const {
  DRAKE_DEMAND(tree != nullptr);
  const bool reverse = mobod.is_reversed();
  // These are the joint's parent and child frames, but adjusted for reversal
  // to locate them on the inboard and outboard bodies. Reversed means
  // outboard==parent and inboard==child; normally outboard==child and
  // inboard==parent.
  const auto [Jin, Jout] = this->tree_frames(reverse);
  // The offset from the inboard frame to the outboard one, which is X_JcJp =
  // (X_JpJc)⁻¹ in the reversed case.
  const math::RigidTransform<double> X_JinJout =
      reverse ? X_JpJc_.inverse() : X_JpJc_;

  // M is the outboard frame; F is colocated with the outboard frame but fixed
  // to the inboard body, so that X_FM is the identity (see above).
  const Frame<T>* M = Jout;
  const Frame<T>* F =
      X_JpJc_.IsExactlyIdentity()
          ? Jin
          : &tree->AddEphemeralFrame(std::make_unique<FixedOffsetFrame<T>>(
                this->MakeUniqueOffsetFrameName(*Jin, "F"), *Jin, X_JinJout,
                this->model_instance()));

  auto weld_mobilizer =
      std::make_unique<internal::WeldMobilizer<T>>(mobod, *F, *M);
  return weld_mobilizer;
}

}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::multibody::WeldJoint);
