#pragma once

#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/multibody/multibody_tree/fixed_offset_frame.h"
#include "drake/multibody/multibody_tree/mobilizer.h"
#include "drake/multibody/multibody_tree/multibody_tree_indexes.h"
#include "drake/multibody/multibody_tree/rigid_body.h"
#include "drake/systems/framework/context.h"

namespace drake {
namespace multibody {

namespace internal {
// This is a class used by MultibodyTree internal's to create the internal
// representation, or model, for a particular joint object.
template <typename T>
class JointModelBuilder;
}

/// A %Joint allows to model a kinematical condition that restricts the relative
/// motion between two rigid bodies.
/// The two rigid bodies connected by a %Joint object are refered to as the
/// _parent_ and _child_ bodies, though a %Joint object does not introduce any
/// particular parent/child relationship. That is, the parent (child) body is
/// not neccessarily the inboard (outboard) body in a MultibodyTree model.
/// A %Joint is a model of a physical kinematic contraint between two rigid
/// bodies, a constraint that in the real physical system does not even allude
/// to the ordering of the bodies.
///
/// In Drake we define a frame F rigidly attached to the parent body P with pose
/// `X_PF` and a frame M rigidly attached to the child body B with pose `X_BM`.
/// A %Joint object specifies a kinematic relation between frames F and M,
/// which in turns imposes a kinematic relation between bodies P and B.
///
/// Typical joints include the ball joint, to allow unrestricted rotations about
/// a given point, the revolute joint, that constraints two bodies to rotate
/// about a given common axis, among others.
///
/// Consider the following example to build a simple pendulum system:
///
/// @code
/// MultibodyTree<double> model;
/// // ... Code here to setup quantities below as mass, com, X_BP, etc. ...
/// const Body<double>& pendulum =
///   model.AddBody<RigidBody>(SpatialInertia<double>(mass, com, unit_inertia));
/// // We will connect the pendulum body to the world using a RevoluteJoint.
/// // In this simple case the parent body P is the model's world body and frame
/// // F coincides with the world frame, i.e. X_PF is the identity
/// // transformation.
/// // Additionally, wee need to specify the pose of frame M on the pendulum's
/// // body frame B.
/// // Say we declared and initialized X_BM...
/// const RevoluteJoint<double>& pin =
///   model.AddJoint<RevoluteJoint>(
///     "PinJoint",             /* joint name */
///     model.get_world_body(), /* parent body */
///     Isometry3d::Identity(), /* frame F IS the world frame W */
///     pendulum,               /* child body, the pendulum */
///     X_BM,                   /* pose of frame M in the body frame B */
///     Vector3d::UnitZ()       /* revolute axis in this case */));
/// @endcode
///
/// @tparam T The scalar type. Must be a valid Eigen scalar.
template <typename T>
class Joint : public MultibodyTreeElement<Joint<T>, JointIndex>  {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(Joint)

  /// Creates a joint between two bodies which imposes a given kinematic
  /// relation between frame F rigidly attached with the parent body P and frame
  /// M rigidly attached with the child body B. See this class's documentation
  /// for further details.
  ///
  /// @param[in] name
  ///   A string with a name identifying `this` joint.
  /// @param[in] parent_body
  ///   One of the rigid bodies connected by this joint.
  /// @param[in] X_PF
  ///   The pose of frame F rigidly attached to the parent body, measured in
  ///   the frame P of that body.
  /// @param[in] child_body
  ///   One of the rigid bodies connected by this joint.
  /// @param[in] X_BM
  ///   The pose of frame M rigidly attached to the child body, measured in
  ///   the frame B of that body.
  Joint(const std::string& name,
        const RigidBody<T>& parent_body, const Isometry3<double>& X_PF,
        const RigidBody<T>& child_body, const Isometry3<double>& X_BM) :
      name_(name), parent_body_(parent_body), child_body_(child_body),
      X_PF_(X_PF), X_BM_(X_BM) {}

  virtual ~Joint() {}

  /// Returns the name of this joint.
  const std::string& get_name() const { return name_; }

  /// Returns a const reference to the parent body P.
  const RigidBody<T>& get_parent_body() const { return parent_body_; }

  /// Returns a const reference to the child body B.
  const RigidBody<T>& get_child_body() const { return child_body_; }

  /// Returns a const reference to the frame F attached on the parent body P.
  const Frame<T>& get_frame_on_parent() const {
    // If a joint is added with MultibodyTree::AddJoint(), inboard_frame_ is
    // guaranteed to be a valid pointer. This is here to avoid users from, for
    // instance, calling this method on stack allocated objects (not allowed).
    DRAKE_DEMAND(inboard_frame_ != nullptr);
    return *inboard_frame_;
  }

  /// Returns a const reference to the frame M attached on the child body B.
  const Frame<T>& get_frame_on_child() const {
    // If a joint is added with MultibodyTree::AddJoint(), outboard_frame_ is
    // guaranteed to be a valid pointer. This is here to avoid users from, for
    // instance, calling this method on stack allocated objects (not allowed).
    DRAKE_DEMAND(outboard_frame_ != nullptr);
    return *outboard_frame_;
  }

  /// Returns the pose `X_PF` of the frame F attached on the parent body, as
  /// measured in that body's frame P.
  const Isometry3<double>& get_frame_on_parent_pose() const {
    return X_PF_;
  }

  /// Returns the pose `X_BM` of the frame M attached on the child body, as
  /// measured in that body's frame B.
  const Isometry3<double>& get_frame_on_child_pose() const {
    return X_BM_;
  }

  /// @cond
  // For internal use only.
  // This is called from MultibodyTree::AddJoint() to make and add the
  // inboard/outboard frames for this Joint object.
  // Therefore public API's get_frame_on_parent()/get_frame_on_child() are
  // available immediately after joint creation.
  void MakeInOutFramesAndAdd(MultibodyTree<T>* tree) {
    // Assert this joint is an element of the input tree.
    // This is to avoid users attempting to call this method by hand.
    this->HasThisParentTreeOrThrow(tree);

    // If the pose X_PF (X_BM) of frame F (M) is kEpsilon within being the
    // identity transform, this method defines frame F (M) to be frame P (B).
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
    // TODO(amcastro-tri): add force elements, constraints, bodies.
  };

  // Make any other Joint<U> a friend of Joint<T> so they can make
  // Joint<ToScalar>::JointModel from CloneToScalar<ToScalar>().
  template <typename> friend class Joint;
  // The model does not own the MBT elements, it just keeps references to them.
  struct JointModel {
    /// Default constructor to create an empty model. Used by
    /// Joint::CloneToScalar().
    JointModel() {}

    /// This constructor creates a model for `this` joint from the blueprint
    /// provided.
    explicit JointModel(const BluePrint& blue_print) {
      DRAKE_DEMAND(static_cast<int>(blue_print.mobilizers_.size()) != 0);
      for (const auto& mobilizer : blue_print.mobilizers_) {
        mobilizers_.push_back(mobilizer.get());
      }
    }

    /// Returns the number of mobilizers in this model.
    int get_num_mobilizers() const { return mobilizers_.size(); }

    /// References (raw pointers) to the mobilizers that make part of this
    /// model.
    std::vector<const Mobilizer<T>*> mobilizers_;
    // TODO(amcastro-tri): add force elements, constraints, bodies, etc.
  };

  // Implements MultibodyTreeElement::DoSetTopology(). Joints have no topology
  // though we could require them to have one in the future.
  void DoSetTopology(const MultibodyTreeTopology&) {}

  /// @name Methods to make a clone templated on different scalar types.
  /// @{
  /// Clones this %Joint (templated on T) to a joint templated on `double`.
  virtual std::unique_ptr<Joint<double>> DoCloneToScalar(
      const MultibodyTree<double>& tree_clone) const = 0;

  /// Clones this %Joint (templated on T) to a joint templated on AutoDiffXd.
  virtual std::unique_ptr<Joint<AutoDiffXd>> DoCloneToScalar(
      const MultibodyTree<AutoDiffXd>& tree_clone) const = 0;
  /// @}

  /// This method must be implemented by derived classes in order to provide
  /// JointModelBuilder a BluePring of their internal implementation,
  /// JointModel.
  virtual std::unique_ptr<BluePrint> MakeModelBlueprint() const = 0;

  /// Returns a const reference to the internal model of `this` joint.
  /// @warning The MultibodyTree model must have already been finalized, or
  /// this method will abort.
  const JointModel& get_model() const {
    // The MultibodyTree must have been finalized for the model to be valid.
    DRAKE_DEMAND(this->get_parent_tree().topology_is_valid());
    return *model_;
  }

 private:
  // When a model is created, either by JointModelBuilder or
  // Joint::CloneToScalar(), this method is called to take ownership of the
  // model.
  void OwnModel(std::unique_ptr<JointModel> model) {
    model_ = std::move(model);
  }

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
