#pragma once

#include <memory>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/multibody/multibody_tree/multibody_tree_element.h"
#include "drake/multibody/multibody_tree/multibody_tree_indexes.h"
#include "drake/multibody/multibody_tree/multibody_tree_topology.h"
#include "drake/multibody/multibody_tree/frame.h"

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
/// does it need to be aligned with the body's principal axes, even though it
/// is a common use case found in practice.
/// For flexible bodies, %BodyFrame provides a representation for the body's
/// reference frame. The flexible degrees of freedom associated with a flexible
/// body describe the body's deformation in this frame. Therefore, the motion of
/// a flexible body is defined by the motion of its %BodyFrame, or reference
/// frame, plus the motion of the material points on the body with respect to
/// its %BodyFrame.
///
/// A %BodyFrame and Body are tightly coupled concepts; neither makes sense
/// without the other. Therefore, a %BodyFrame instance is constructed in
/// conjunction with its Body in the corresponding Create() method and cannot be
/// constructed anywhere else. However, users can still access the frame
/// associated with a body, see Body::get_body_frame().
/// This access is more than a convenience; it allows users to specify
/// mobilizers between a body frame and any other Frame in the multibody
/// tree.
///
/// @tparam T The scalar type. Must be a valid Eigen scalar.
template <typename T>
class BodyFrame : public Frame<T> {
  // Body<T> and BodyFrame<T> are natural allies. A BodyFrame object is created
  // every time a Body object is created and they are associated with each
  // other. Moreover, BodyFrame objects can *only* be created by Body objects
  // through their protected method Body::CreateBodyFrame().
  friend class Body<T>;
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(BodyFrame)

 public:
  // Implementation for MultibodyTreeElement::Compile().
  void Compile(const MultibodyTree<T>& tree) final {}

 private:
  // Only Body objects can create BodyFrame objects since Body is a friend of
  // BodyFrame. BodyFrame objects are *only* created from within
  // Body::CreateBodyFrame().
  explicit BodyFrame(const Body<T>& body) : Frame<T>(body) {}
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

  /// Returns a constant reference to the associated BodyFrame.
  const BodyFrame<T>& get_body_frame() const {
    return body_frame_;
  }

  /// Returns a mutable pointer to the associated BodyFrame.
  BodyFrame<T>* get_mutable_body_frame() {
    return &body_frame_;
  }

  /// At MultibodyTree::Compile() time, each body retrieves its topology
  /// from the parent MultibodyTree.
  void Compile(const MultibodyTree<T>& tree) final {
    topology_ = tree.get_topology().bodies[this->get_index()];
  }

 protected:
  // Default constructor. Only sub-classes can use it.
  Body() : body_frame_(*this) {}

 private:
  // Body frame associated with this body.
  BodyFrame<T> body_frame_;

  // The internal bookkeeping topology struct used by MultibodyTree.
  BodyTopology topology_;
};

}  // namespace multibody
}  // namespace drake
