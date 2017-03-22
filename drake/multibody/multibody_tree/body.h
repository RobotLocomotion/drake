#pragma once

#include <memory>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/multibody/multibody_tree/multibody_tree_element.h"
#include "drake/multibody/multibody_tree/multibody_tree_indexes.h"
#include "drake/multibody/multibody_tree/material_frame.h"

namespace drake {
namespace multibody {

// Body<T> forward declaration for BodyFrame<T>.
template<typename T> class Body;

/// All Body objects, regardless of whether they represent rigid bodies or
/// flexible bodies, have a %BodyFrame associated with them (also referred to as
/// a _reference frame_ in the literature for flexible body mechanics with FEM).
/// These body frames can be thought of as a set of three orthogonal axes
/// forming a right-handed orthogonal basis located at a point called the
/// frame's origin. These %BodyFrame objects translate and rotate with their
/// associated body and therefore their location and orientation is a function
/// of time.
/// For RigidBody objects %BodyFrame represents the frame in which their center
/// of mass and rotational inertia are provided. The %BodyFrame associated with
/// a body does not necessarily need to be located at its center of mass nor
/// it needs to be aligned with the body's principal axes, even though it is a
/// common use case found in practice.
/// For flexible bodies, %BodyFrame provides a representation for the body's
/// reference frame. The flexible degrees of freedom associated with a flexible
/// body describe the body's deformation in this frame. Therefore, the motion of
/// a flexible body is defined by the motion of its %BodyFrame, or reference
/// frame, plus the motion of the material points on the body with respect to
/// its %BodyFrame.
///
/// This class %BodyFrame provides a representaion for the frame associated with
/// a body. %BodyFrame objects are not meant to be instantiated directly by the
/// user and therefore there are no public constructors for this class.
/// The %BodyFrame for a Body gets constructed behind the scenes when the user
/// creates a new Body through ones of its `Create()` factory methods, see
/// RigidBody::Create() for an example of a Body `Create()` factory method.
/// However, users can still access the frame associated with a body, see
/// Body::get_body_frame(). This access is more than a convenience, but it
/// allows users to specify mobilizers between a body frame and any other
/// MaterialFrame in the multibody tree.
///
/// @tparam T The scalar type. Must be a valid Eigen scalar.
template <typename T>
class BodyFrame : public MaterialFrame<T> {
  // Body<T> and BodyFrame<T> are natural allies. A BodyFrame object is created
  // every time a Body object is created and they are both associated with each
  // other. Moreover, BodyFrame objects can *only* be created by Body objects
  // through their protected method Body::CreateBodyFrame().
  friend class Body<T>;
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(BodyFrame)

 public:
  // Implementation for MultibodyTreeElement::Compile().
  void Compile() final {}

 private:
  // Only Body objects can create BodyFrame objects since Body is a friend of
  // BodyFrame. BodyFrame objects are *only* created from within
  // Body::CreateBodyFrame().
  explicit BodyFrame(BodyIndex body_index) : MaterialFrame<T>(body_index) {}
};

// Forward declarations for Body<T>.
template<typename T> class MultibodyTree;

/// This class provides the general abstraction of a body with an API that
/// makes no assumption about whether a body is rigid or deformable and neither
/// does it make any assumptions about the underlying physical model or
/// approximation.
/// As an element or component of a MultibodyTree, a body is a
/// MultibodyTreeElement, and therefore it has a unique index of type BodyIndex
/// within the multibody tree it belongs to.
///
/// @tparam T The scalar type. Must be a valid Eigen scalar.
template <typename T>
class Body : public MultibodyTreeElement<Body<T>, BodyIndex> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(Body)

  /// Returns the number of generalized positions describing flexible
  /// deformations for this body. A rigid body will therefore return zero.
  virtual int get_num_flexible_positions() const = 0;

  /// Returns the number of generalized velocities describing flexible
  /// deformations for this body. A rigid body will therefore return zero.
  virtual int get_num_flexible_velocities() const = 0;

  /// Returns a constant reference to the associated BodyFrame in the parent
  /// MultibodyTree.
  const BodyFrame<T>& get_body_frame() const {
    return this->get_parent_tree().get_body_frame(*this);
  }

  /// Returns the unique identifier of the body frame associated to this body in
  /// its parent MultibodyTree.
  FrameIndex get_body_frame_index() const { return body_frame_index_; }

  /// At MultibodyTree::Compile() time, each body will retrieve its topology
  /// from the parent MultibodyTree.
  virtual void Compile() {}

 protected:
  // Default constructor. Only sub-classes can use it.
  Body() = default;

  // Only sub-classes' Create() methods can set the body frame index.
  void set_body_frame_index(FrameIndex index) { body_frame_index_ = index; }

  // This method is called within the Create() methods of a sub-class inheriting
  // from Body to create the BodyFrame associated with this body.
  // Since Body<T> is a friend of BodyFrame<T>, it has access to its private
  // constructor.
  // Even though this method is called within a body which already has a valid
  // parent multibody tree, its parent tree is passed as an argument in order to
  // have mutable access to it and add a new material frame for this body.
  const BodyFrame<T>& CreateBodyFrame(MultibodyTree<T>* tree) const {
    this->HasParentTreeOrThrow();
    this->HasThisParentTreeOrThrow(tree);
    // Notice that here we cannot use std::make_unique since constructors are
    // made private to avoid users creating bodies by other means other than
    // calling Create().
    // However we can still create a unique_ptr as below where ownership is
    // clear
    return *tree->AddBodyFrame(
        std::unique_ptr<BodyFrame<T>>(new BodyFrame<T>(this->get_index())));
  }

 private:
  // Unique index to the associated BodyFrame in the parent MultibodyTree.
  // TypeSafeIndex objects must be initialized.
  FrameIndex body_frame_index_{0};
};

}  // namespace multibody
}  // namespace drake
