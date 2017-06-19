#pragma once

#include <memory>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/common/unused.h"
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
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(BodyFrame)

  Isometry3<T> CalcBodyPoseInThisFrame(
      const systems::Context<T>& context) const final {
    unused(context);
    return Isometry3<T>::Identity();
  }

  Isometry3<T> CalcOffsetPoseInBody(
      const systems::Context<T>& context,
      const Isometry3<T>& X_FQ) const final {
    unused(context);
    return X_FQ;
  }

  Isometry3<T> CalcBodyPoseInOtherFrame(
      const systems::Context<T>& context,
      const Isometry3<T>& X_QF) const final {
    unused(context);
    return X_QF;
  }

 private:
  // Body<T> and BodyFrame<T> are natural allies. A BodyFrame object is created
  // every time a Body object is created and they are associated with each
  // other.
  friend class Body<T>;

  // Only Body objects can create BodyFrame objects since Body is a friend of
  // BodyFrame.
  explicit BodyFrame(const Body<T>& body) : Frame<T>(body) {}
};

// Forward declarations for Body<T>.
template<typename T> class MultibodyTree;

/// @cond
// Internal implementation details. Users should not access implementations
// in this namespace.
namespace internal {
template <typename T>
// Attorney-Client idiom to grant MultibodyTree access to a selected set of
// private methods in Body.
// BodyAttorney serves as a "proxy" to the Body class but only providing an
// interface to a selected subset of methods that should be accessible to
// MultibodyTree. These methods are related to the construction and finalize
// stage of the multibody system.
class BodyAttorney {
 private:
  // MultibodyTree keeps a list of mutable pointers to each of the body frames
  // in the system and therefore it needs mutable access.
  // Notice this method is private and therefore users do not have access to it
  // even in the rare event they'd attempt to peek into the "internal::"
  // namespace.
  static BodyFrame<T>& get_mutable_body_frame(Body<T>* body) {
    return body->get_mutable_body_frame();
  }
  friend class MultibodyTree<T>;
};
}  // namespace internal
/// @endcond

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

 private:
  // Only friends of BodyAttorney (i.e. MultibodyTree) have access to a selected
  // set of private Body methods.
  friend class internal::BodyAttorney<T>;

  // Implementation for MultibodyTreeElement::DoSetTopology().
  // At MultibodyTree::Finalize() time, each body retrieves its topology
  // from the parent MultibodyTree.
  void DoSetTopology(const MultibodyTreeTopology& tree_topology) final {
    topology_ = tree_topology.get_body(this->get_index());
    body_frame_.SetTopology(tree_topology);
  }

  // MultibodyTree has access to the mutable BodyFrame through BodyAttorney.
  BodyFrame<T>& get_mutable_body_frame() {
    return body_frame_;
  }

  // Body frame associated with this body.
  BodyFrame<T> body_frame_;

  // The internal bookkeeping topology struct used by MultibodyTree.
  BodyTopology topology_;
};

}  // namespace multibody
}  // namespace drake
