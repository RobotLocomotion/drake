#pragma once

#include <memory>
#include <vector>

#include "drake/common/autodiff.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/unused.h"
#include "drake/multibody/multibody_tree/frame.h"
#include "drake/multibody/multibody_tree/multibody_tree_element.h"
#include "drake/multibody/multibody_tree/multibody_tree_indexes.h"
#include "drake/multibody/multibody_tree/multibody_tree_topology.h"
#include "drake/multibody/multibody_tree/spatial_inertia.h"

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
class BodyFrame final : public Frame<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(BodyFrame)

  Isometry3<T> CalcPoseInBodyFrame(
      const systems::Context<T>&) const override {
    return Isometry3<T>::Identity();
  }

  Isometry3<T> CalcOffsetPoseInBody(
      const systems::Context<T>&,
      const Isometry3<T>& X_FQ) const override {
    return X_FQ;
  }

 protected:
  // Frame<T>::DoCloneToScalar() overrides.
  std::unique_ptr<Frame<double>> DoCloneToScalar(
      const MultibodyTree<double>& tree_clone) const override;

  std::unique_ptr<Frame<AutoDiffXd>> DoCloneToScalar(
      const MultibodyTree<AutoDiffXd>& tree_clone) const override;

 private:
  // Body<T> and BodyFrame<T> are natural allies. A BodyFrame object is created
  // every time a Body object is created and they are associated with each
  // other.
  friend class Body<T>;

  // Make BodyFrame templated on any other scalar type a friend of
  // BodyFrame<T> so that CloneToScalar<ToAnyOtherScalar>() can access
  // private methods from BodyFrame<T>.
  template <typename> friend class BodyFrame;

  // Only Body objects can create BodyFrame objects since Body is a friend of
  // BodyFrame.
  explicit BodyFrame(const Body<T>& body) : Frame<T>(body) {}

  // Helper method to make a clone templated on any other scalar type.
  // This method holds the common implementation for the different overrides to
  // DoCloneToScalar().
  template <typename ToScalar>
  std::unique_ptr<Frame<ToScalar>> TemplatedDoCloneToScalar(
      const MultibodyTree<ToScalar>& tree_clone) const;
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

  /// Returns the index of the node in the underlying tree structure of
  /// the parent MultibodyTree to which this body belongs.
  BodyNodeIndex get_node_index() const {
    return topology_.body_node;
  }

  /// Returns the mass of this body stored in `context`.
  virtual T get_mass(const MultibodyTreeContext<T> &context) const = 0;

  /// Computes the center of mass `p_BoBcm_B` (or `p_Bcm` for short) of this
  /// body measured from this body's frame origin `Bo` and expressed in the body
  /// frame B.
  virtual const Vector3<T> CalcCenterOfMassInBodyFrame(
      const MultibodyTreeContext<T>& context) const = 0;

  /// Computes the SpatialInertia `I_BBo_B` of `this` body about its frame
  /// origin `Bo` (not necessarily its center of mass) and expressed in its body
  /// frame `B`.
  /// In general, the spatial inertia of a body is a function of state.
  /// Consider for instance the case of a flexible body for which its spatial
  /// inertia in the body frame depends on the generalized coordinates
  /// describing its state of deformation. As a particular case, the spatial
  /// inertia of a RigidBody in its body frame is constant.
  virtual SpatialInertia<T> CalcSpatialInertiaInBodyFrame(
      const MultibodyTreeContext<T>& context) const = 0;

  /// NVI (Non-Virtual Interface) to DoCloneToScalar() templated on the scalar
  /// type of the new clone to be created. This method is mostly intended to be
  /// called by MultibodyTree::CloneToScalar(). Most users should not call this
  /// clone method directly but rather clone the entire parent MultibodyTree if
  /// needed.
  /// @sa MultibodyTree::CloneToScalar()
  template <typename ToScalar>
  std::unique_ptr<Body<ToScalar>> CloneToScalar(
  const MultibodyTree<ToScalar>& tree_clone) const {
    return DoCloneToScalar(tree_clone);
  }

 protected:
  /// @name Methods to make a clone templated on different scalar types.
  ///
  /// These methods are meant to be called by MultibodyTree::CloneToScalar()
  /// when making a clone of the entire tree or a new instance templated on a
  /// different scalar type. The only const argument to these methods is the
  /// new MultibodyTree clone under construction. Specific %Body subclasses
  /// might specify a number of prerequisites on the cloned tree and therefore
  /// require it to be at a given state of cloning (for instance requiring that
  /// the cloned tree already contains all the frames in the world as in the
  /// original tree.) See MultibodyTree::CloneToScalar() for a list of
  /// prerequisites that are guaranteed to be satisfied during the cloning
  /// process.
  ///
  /// @{

  /// Clones this %Body (templated on T) to a body templated on `double`.
  virtual std::unique_ptr<Body<double>> DoCloneToScalar(
      const MultibodyTree<double>& tree_clone) const = 0;

  /// Clones this %Body (templated on T) to a body templated on AutoDiffXd.
  virtual std::unique_ptr<Body<AutoDiffXd>> DoCloneToScalar(
      const MultibodyTree<AutoDiffXd>& tree_clone) const = 0;

  /// @}

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
