#pragma once

#include <limits>
#include <string>

#include "drake/common/drake_copyable.h"
#include "drake/multibody/multibody_tree/fixed_offset_frame.h"
#include "drake/multibody/multibody_tree/mobilizer.h"
#include "drake/multibody/multibody_tree/multibody_tree_indexes.h"
#include "drake/multibody/multibody_tree/rigid_body.h"
#include "drake/systems/framework/context.h"

namespace drake {
namespace multibody {

template <typename T>
class Joint : public MultibodyTreeElement<Joint<T>, JointIndex>  {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(Joint)

  Joint(const std::string& name,
        const RigidBody<T>& iboard_body, const Isometry3<double> X_PF,
        const RigidBody<T>& outboard_body, const Isometry3<double> X_BM) :
      name_(name), inboard_body_(iboard_body), outboard_body_(outboard_body),
      X_PF_(X_PF), X_BM_(X_BM) {}

  virtual ~Joint() {}

  const std::string& get_name() const { return name_; }

  const RigidBody<T>& get_inboard_body() const { return inboard_body_; }
  const RigidBody<T>& get_outboard_body() const { return outboard_body_; }

  const Frame<T>& get_inboard_frame() const {
    // If a joint is added with MultibodyTree::AddJoint(), inboard_frame_ is
    // guaranteed to be a valid pointer. This is here to avoid users from, for
    // instance, calling this method on stack allocated objects (not allowed).
    DRAKE_DEMAND(inboard_frame_ != nullptr);
    return *inboard_frame_;
  }

  const Frame<T>& get_outboard_frame() const {
    // If a joint is added with MultibodyTree::AddJoint(), outboard_frame_ is
    // guaranteed to be a valid pointer. This is here to avoid users from, for
    // instance, calling this method on stack allocated objects (not allowed).
    DRAKE_DEMAND(outboard_frame_ != nullptr);
    return *outboard_frame_;
  }

  const Isometry3<double>& get_inboard_frame_pose() const {
    return X_PF_;
  }

  const Isometry3<double>& get_outboard_frame_pose() const {
    return X_BM_;
  }
  /// @cond

  // For internal use only.
  // This is called from MultibodyTree::AddJoint() to make and add the
  // inboard/outboard frames for this Joint object.
  // Therefore public API's get_inboard_frame()/get_outboard_frame() are
  // available immediately with no side effects.
  void MakeInOutFramesAndAdd(MultibodyTree<T>* tree) {
    // Assert this joint is an element of the input tree.
    // This is to avoid users attempting to call this method by hand.
    this->HasThisParentTreeOrThrow(tree);

    const double kEpsilon = std::numeric_limits<double>::epsilon();

    // Define the joint's inboard frame.
    // If X_PF is the identity transformation, then the user meant to say that
    // the inboard frame F IS the inboard body frame P.
    if (get_inboard_frame_pose().matrix().isIdentity(kEpsilon)) {
      inboard_frame_ = &get_inboard_body().get_body_frame();
    } else {
      inboard_frame_ = &tree->template AddFrame<FixedOffsetFrame>(
          get_inboard_body(), get_inboard_frame_pose());
    }

    // Define the joint's outboard frame.
    // If X_BM is the identity transformation, then the user meant to say that
    // the outboard frame M IS the outboard body frame B.
    if (get_outboard_frame_pose().matrix().isIdentity(kEpsilon)) {
      outboard_frame_ = &get_outboard_body().get_body_frame();
    } else {
      outboard_frame_ = &tree->template AddFrame<FixedOffsetFrame>(
          get_outboard_body(), get_outboard_frame_pose());
    }
  }

  // For internal use only.
  void MakeModelAndAdd(MultibodyTree<T>* tree) {
    // Assert this joint is an element of the input tree.
    // This is to avoid users attempting to call this method by hand.
    this->HasThisParentTreeOrThrow(tree);

    // These should pass if MakeInOutFramesAndAdd() was already called from
    // within MultibodyTree::AddJoint() (which by the way is the only allowed
    // mechanism to add joints).
    DRAKE_DEMAND(inboard_frame_ != nullptr);
    DRAKE_DEMAND(outboard_frame_ != nullptr);

    // Add joint subclass specific model.
    DoMakeModelAndAdd(tree);
  }

  // NVI to DoCloneToScalar() templated on the scalar type of the new clone to
  // be created. This method is intended to be called by
  // MultibodyTree::CloneToScalar().
  template <typename ToScalar>
  std::unique_ptr<Joint<ToScalar>> CloneToScalar(
      const MultibodyTree<ToScalar>& cloned_tree) const {
    return DoCloneToScalar(cloned_tree);
  }
  /// @endcond

 protected:
  // Implements MultibodyTreeElement::DoSetTopology(). Joints have no topology
  // though we could require them to have one in the future.
  void DoSetTopology(const MultibodyTreeTopology& tree) {}

  // Implements MakeModelAndAdd() NVI.
  virtual void DoMakeModelAndAdd(MultibodyTree<T>* tree) = 0;

  /// @name Methods to make a clone templated on different scalar types.
  /// @{
  /// Clones this %Joint (templated on T) to a joint templated on `double`.
  virtual std::unique_ptr<Joint<double>> DoCloneToScalar(
      const MultibodyTree<double>& tree_clone) const = 0;

  /// Clones this %Joint (templated on T) to a joint templated on AutoDiffXd.
  virtual std::unique_ptr<Joint<AutoDiffXd>> DoCloneToScalar(
      const MultibodyTree<AutoDiffXd>& tree_clone) const = 0;
  /// @}

 private:
  std::string name_;
  const RigidBody<T>& inboard_body_;
  const RigidBody<T>& outboard_body_;

  // Inboard/outboard frame pointers are set by Joint::MakeAndAddModel() called
  // from within MultibodyTree::AddJoint().
  const Frame<T>* inboard_frame_{nullptr};
  const Frame<T>* outboard_frame_{nullptr};

  // The pose of the inboard frame F rigidly attached to body P.
  Isometry3<double> X_PF_;

  // The pose of the outboard frame M rigidly attached to body B.
  Isometry3<double> X_BM_;
};

}  // namespace multibody
}  // namespace drake
