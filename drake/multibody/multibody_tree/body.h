#pragma once

#include <memory>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/multibody/multibody_tree/frame.h"
#include "drake/multibody/multibody_tree/multibody_tree_element.h"
#include "drake/multibody/multibody_tree/multibody_tree_indexes.h"
#include "drake/multibody/multibody_tree/multibody_tree_topology.h"

namespace drake {
namespace multibody {

// Forward declaration for BodyFrame<T>.
template<typename T> class Body;

/// A %BodyFrame is a material Frame that serves as the unique reference frame
/// for a Body.
///
/// Each Body B, regardless of whether it represents a rigid body or a
/// flexible body, has a unique body frame for which we use the same symbol B
/// (with meaning clear from context). The body frame is also referred to as
/// a _reference frame_ in the literature for flexible body mechanics modeling
/// using the Finite Element Method. All properties of a body are defined with
/// respect to its body frame, including its mass properties and attachment
/// locations for joints, constraints, actuators, geometry and so on. Run time
/// motion of the body is defined with respect to the motion of its body frame.
/// We represent a body frame by a %BodyFrame object that is created whenever a
/// Body is constructed and is owned by the Body.
///
/// Note that the %BodyFrame associated with
/// a body does not necessarily need to be located at its center of mass nor
/// does it need to be aligned with the body's principal axes, although, in
/// practice, it frequently is.
/// For flexible bodies, %BodyFrame provides a representation for the body's
/// reference frame. The flexible degrees of freedom associated with a flexible
/// body describe the body's deformation in this frame. Therefore, the motion of
/// a flexible body is defined by the motion of its %BodyFrame, or reference
/// frame, plus the motion of the material points on the body with respect to
/// its %BodyFrame.
///
/// A %BodyFrame and Body are tightly coupled concepts; neither makes sense
/// without the other. Therefore, a %BodyFrame instance is constructed in
/// conjunction with its Body and cannot be
/// constructed anywhere else. However, you can still access the frame
/// associated with a body, see Body::get_body_frame().
/// This access is more than a convenience; you can use the %BodyFrame to
/// define other frames on the body and to attach other multibody elements
/// to it.
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

 private:
  // Only Body objects can create BodyFrame objects since Body is a friend of
  // BodyFrame.
  explicit BodyFrame(const Body<T>& body) : Frame<T>(body) {}
};

// Forward declarations for Body<T>.
template<typename T> class MultibodyTree;

/// %Body provides the general abstraction of a body with an API that
/// makes no assumption about whether a body is rigid or deformable and neither
/// does it make any assumptions about the underlying physical model or
/// approximation.
/// As an element or component of a MultibodyTree, a body is a
/// MultibodyTreeElement, and therefore it has a unique index of type BodyIndex
/// within the multibody tree it belongs to.
///
/// A %Body contains a unique BodyFrame; see BodyFrame class documentation for
/// more information.
///
/// @tparam T The scalar type. Must be a valid Eigen scalar.
template <typename T>
class Body : public MultibodyTreeElement<Body<T>, BodyIndex> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(Body)

  /// Creates a %Body with a BodyFrame associated with it.
  Body() : body_frame_(*this) {}

  /// Returns the number of generalized positions q describing flexible
  /// deformations for this body. A rigid body will therefore return zero.
  virtual int get_num_flexible_positions() const = 0;

  /// Returns the number of generalized velocities v describing flexible
  /// deformations for this body. A rigid body will therefore return zero.
  virtual int get_num_flexible_velocities() const = 0;

  /// Returns a const reference to the associated BodyFrame.
  const BodyFrame<T>& get_body_frame() const {
    return body_frame_;
  }

  /// Returns a mutable pointer to the associated BodyFrame.
  BodyFrame<T>* get_mutable_body_frame() {
    return &body_frame_;
  }

 private:
  // Implementation for MultibodyTreeElement::DoCompile().
  // At MultibodyTree::Compile() time, each body retrieves its topology
  // from the parent MultibodyTree.
  void DoCompile(const MultibodyTree<T>& tree) final {
    topology_ = tree.get_topology().bodies[this->get_index()];
    body_frame_.Compile(tree);
  }

  // Body frame associated with this body.
  BodyFrame<T> body_frame_;

  // The internal bookkeeping topology struct used by MultibodyTree.
  BodyTopology topology_;

};

}  // namespace multibody
}  // namespace drake
