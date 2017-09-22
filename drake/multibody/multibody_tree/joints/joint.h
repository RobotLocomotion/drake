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

namespace internal {
template <typename T>
class JointModelBuilder;
}

template <typename T>
class Joint : public MultibodyTreeElement<Joint<T>, JointIndex>  {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(Joint)

  Joint(const std::string& name,
        const RigidBody<T>& parent_body, const Isometry3<double>& X_PF,
        const RigidBody<T>& child_body, const Isometry3<double>& X_BM) :
      name_(name), parent_body_(parent_body), child_body_(child_body),
      X_PF_(X_PF), X_BM_(X_BM) {}

  virtual ~Joint() {}

  const std::string& get_name() const { return name_; }

  const RigidBody<T>& get_parent_body() const { return parent_body_; }
  const RigidBody<T>& get_child_body() const { return child_body_; }

  const Frame<T>& get_frame_on_parent() const {
    // If a joint is added with MultibodyTree::AddJoint(), inboard_frame_ is
    // guaranteed to be a valid pointer. This is here to avoid users from, for
    // instance, calling this method on stack allocated objects (not allowed).
    DRAKE_DEMAND(inboard_frame_ != nullptr);
    return *inboard_frame_;
  }

  const Frame<T>& get_frame_on_child() const {
    // If a joint is added with MultibodyTree::AddJoint(), outboard_frame_ is
    // guaranteed to be a valid pointer. This is here to avoid users from, for
    // instance, calling this method on stack allocated objects (not allowed).
    DRAKE_DEMAND(outboard_frame_ != nullptr);
    return *outboard_frame_;
  }

  const Isometry3<double>& get_frame_on_parent_pose() const {
    return X_PF_;
  }

  const Isometry3<double>& get_frame_on_child_pose() const {
    return X_BM_;
  }

  /// @cond
  // For internal use only.
  // This is called from MultibodyTree::AddJoint() to make and add the
  // inboard/outboard frames for this Joint object.
  // Therefore public API's get_frame_on_parent()/get_frame_on_child() are
  // available immediately with no side effects.
  void MakeInOutFramesAndAdd(MultibodyTree<T>* tree) {
    // Assert this joint is an element of the input tree.
    // This is to avoid users attempting to call this method by hand.
    this->HasThisParentTreeOrThrow(tree);

    const double kEpsilon = std::numeric_limits<double>::epsilon();

    // Define the joint's inboard frame.
    // If X_PF is the identity transformation, then the user meant to say that
    // the inboard frame F IS the inboard body frame P.
    if (get_frame_on_parent_pose().matrix().isIdentity(kEpsilon)) {
      inboard_frame_ = &get_parent_body().get_body_frame();
    } else {
      inboard_frame_ = &tree->template AddFrame<FixedOffsetFrame>(
          get_parent_body(), get_frame_on_parent_pose());
    }

    // Define the joint's outboard frame.
    // If X_BM is the identity transformation, then the user meant to say that
    // the outboard frame M IS the outboard body frame B.
    if (get_frame_on_child_pose().matrix().isIdentity(kEpsilon)) {
      outboard_frame_ = &get_child_body().get_body_frame();
    } else {
      outboard_frame_ = &tree->template AddFrame<FixedOffsetFrame>(
          get_child_body(), get_frame_on_child_pose());
    }
  }

  // NVI to DoCloneToScalar() templated on the scalar type of the new clone to
  // be created. This method is intended to be called by
  // MultibodyTree::CloneToScalar().
  template <typename ToScalar>
  std::unique_ptr<Joint<ToScalar>> CloneToScalar(
      const MultibodyTree<ToScalar>& tree_clone) const {
    std::unique_ptr<Joint<ToScalar>> joint_clone = DoCloneToScalar(tree_clone);

    // Make the JointModel clone.
    auto model_clone = std::make_unique<typename Joint<ToScalar>::JointModel>();
    const Mobilizer<ToScalar>* mobilizer_clone =
        &tree_clone.get_variant(*this->get_model().mobilizers_[0]);
    model_clone->mobilizers_.push_back(mobilizer_clone);
    joint_clone->OwnModel(std::move(model_clone));

    return std::move(joint_clone);
  }
  /// @endcond

 protected:
  // JointModelBuilder is a friend so that it can access the
  // Joint<T>::BluePrint and protected method MakeModelBlueprint().
  friend class internal::JointModelBuilder<T>;
  struct BluePrint {
    std::vector<std::unique_ptr<Mobilizer<T>>> mobilizers_;
  };

  // Make any other Joint<U> a friend of Joint<T> so they can make
  // Joint<ToScalar>::JointModel from CloneToScalar<ToScalar>().
  template <typename> friend class Joint;
  // The model does not own the MBT elements, it just keeps references to them.
  struct JointModel {
    JointModel() {}
    JointModel(const BluePrint& blue_print) {
      // For now only allow models to have a single mobilizer.
      DRAKE_DEMAND(static_cast<int>(blue_print.mobilizers_.size()) == 1);
      mobilizers_.push_back(blue_print.mobilizers_[0].get());
    }

    int get_num_mobilizers() const { return mobilizers_.size(); }

    std::vector<const Mobilizer<T>*> mobilizers_;
    // TODO(amcastro-tri): add force elements, constraints, bodies, etc.
  };

  // Implements MultibodyTreeElement::DoSetTopology(). Joints have no topology
  // though we could require them to have one in the future.
  void DoSetTopology(const MultibodyTreeTopology& tree) {}

  /// @name Methods to make a clone templated on different scalar types.
  /// @{
  /// Clones this %Joint (templated on T) to a joint templated on `double`.
  virtual std::unique_ptr<Joint<double>> DoCloneToScalar(
      const MultibodyTree<double>& tree_clone) const = 0;

  /// Clones this %Joint (templated on T) to a joint templated on AutoDiffXd.
  virtual std::unique_ptr<Joint<AutoDiffXd>> DoCloneToScalar(
      const MultibodyTree<AutoDiffXd>& tree_clone) const = 0;
  /// @}

  virtual std::unique_ptr<BluePrint> MakeModelBlueprint() const = 0;
  void OwnModel(std::unique_ptr<JointModel> model) {
    model_ = std::move(model);
  }
  const JointModel& get_model() const { return *model_; }

 private:
  std::string name_;
  const RigidBody<T>& parent_body_;
  const RigidBody<T>& child_body_;

  // Inboard/outboard frame pointers are set by Joint::MakeAndAddModel() called
  // from within MultibodyTree::AddJoint().
  const Frame<T>* inboard_frame_{nullptr};
  const Frame<T>* outboard_frame_{nullptr};

  // The pose of the inboard frame F rigidly attached to body P.
  Isometry3<double> X_PF_;

  // The pose of the outboard frame M rigidly attached to body B.
  Isometry3<double> X_BM_;

  // The Joint<T> implementation:
  std::unique_ptr<JointModel> model_;
};

}  // namespace multibody
}  // namespace drake
