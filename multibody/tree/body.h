#pragma once

#include <memory>
#include <string>
#include <vector>

#include "drake/common/default_scalars.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/unused.h"
#include "drake/multibody/tree/frame.h"
#include "drake/multibody/tree/multibody_element.h"
#include "drake/multibody/tree/multibody_forces.h"
#include "drake/multibody/tree/multibody_tree_indexes.h"
#include "drake/multibody/tree/multibody_tree_topology.h"
#include "drake/multibody/tree/spatial_inertia.h"
#include "drake/systems/framework/context.h"

namespace drake {
namespace multibody {

/// @cond
// Helper macro to throw an exception within methods that should not be called
// pre-finalize.
#define DRAKE_BODY_THROW_IF_NOT_FINALIZED() ThrowIfNotFinalized(__func__)
/// @endcond

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
/// associated with a body, see Body::body_frame().
/// This access is more than a convenience; you can use the %BodyFrame to
/// define other frames on the body and to attach other multibody elements
/// to it.
///
/// @tparam_default_scalar
template <typename T>
class BodyFrame final : public Frame<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(BodyFrame)

  math::RigidTransform<T> CalcPoseInBodyFrame(
      const systems::Context<T>&) const override {
    return math::RigidTransform<T>::Identity();
  }

  math::RotationMatrix<T> CalcRotationMatrixInBodyFrame(
      const systems::Context<T>&) const override {
    return math::RotationMatrix<T>::Identity();
  }

  math::RigidTransform<T> CalcOffsetPoseInBody(
      const systems::Context<T>&,
      const math::RigidTransform<T>& X_FQ) const override {
    return X_FQ;
  }

  math::RotationMatrix<T> CalcOffsetRotationMatrixInBody(
      const systems::Context<T>&,
      const math::RotationMatrix<T>& R_FQ) const override {
    return R_FQ;
  }

  math::RigidTransform<T> GetFixedPoseInBodyFrame() const override {
    return math::RigidTransform<T>::Identity();
  }

  math::RotationMatrix<T> GetFixedRotationMatrixInBodyFrame() const override {
    return math::RotationMatrix<T>::Identity();
  }

  math::RigidTransform<T> GetFixedOffsetPoseInBody(
      const math::RigidTransform<T>& X_FQ) const override {
    return X_FQ;
  }

  math::RotationMatrix<T> GetFixedRotationMatrixInBody(
      const math::RotationMatrix<T>& R_FQ) const override {
    return R_FQ;
  }

 protected:
  // Frame<T>::DoCloneToScalar() overrides.
  std::unique_ptr<Frame<double>> DoCloneToScalar(
      const internal::MultibodyTree<double>& tree_clone) const override;

  std::unique_ptr<Frame<AutoDiffXd>> DoCloneToScalar(
      const internal::MultibodyTree<AutoDiffXd>& tree_clone) const override;

  std::unique_ptr<Frame<symbolic::Expression>> DoCloneToScalar(
      const internal::MultibodyTree<symbolic::Expression>&) const override;

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
  explicit BodyFrame(const Body<T>& body) : Frame<T>(body.name(), body) {}

  // Helper method to make a clone templated on any other scalar type.
  // This method holds the common implementation for the different overrides to
  // DoCloneToScalar().
  template <typename ToScalar>
  std::unique_ptr<Frame<ToScalar>> TemplatedDoCloneToScalar(
      const internal::MultibodyTree<ToScalar>& tree_clone) const;
};

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
  friend class internal::MultibodyTree<T>;
};
}  // namespace internal
/// @endcond

/// %Body provides the general abstraction of a body with an API that
/// makes no assumption about whether a body is rigid or deformable and neither
/// does it make any assumptions about the underlying physical model or
/// approximation.
/// As an element or component of a MultibodyTree, a body is a
/// MultibodyElement, and therefore it has a unique index of type BodyIndex
/// within the multibody tree it belongs to.
///
/// A %Body contains a unique BodyFrame; see BodyFrame class documentation for
/// more information.
///
/// @tparam_default_scalar
template <typename T>
class Body : public MultibodyElement<Body, T, BodyIndex> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(Body)

  /// Creates a %Body named `name` in model instance `model_instance`
  /// with a given `default_mass` and a BodyFrame associated with it.
  Body(const std::string& name, ModelInstanceIndex model_instance,
       double default_mass)
      : MultibodyElement<Body, T, BodyIndex>(model_instance),
        name_(name),
        body_frame_(*this), default_mass_(default_mass) {}

  /// Gets the `name` associated with `this` body.
  const std::string& name() const { return name_; }

  /// Returns the number of generalized positions q describing flexible
  /// deformations for this body. A rigid body will therefore return zero.
  virtual int get_num_flexible_positions() const = 0;

  /// Returns the number of generalized velocities v describing flexible
  /// deformations for this body. A rigid body will therefore return zero.
  virtual int get_num_flexible_velocities() const = 0;

  /// Returns a const reference to the associated BodyFrame.
  const BodyFrame<T>& body_frame() const {
    return body_frame_;
  }

  /// (Advanced) Returns the index of the node in the underlying tree structure
  /// of the parent MultibodyTree to which this body belongs.
  internal::BodyNodeIndex node_index() const {
    return topology_.body_node;
  }

  /// (Advanced) Returns `true` if `this` body is granted 6-dofs by a Mobilizer.
  /// @note A floating body is not necessarily modeled with a quaternion
  /// mobilizer, see has_quaternion_dofs(). Alternative options include a space
  /// XYZ parametrization of rotations, see SpaceXYZMobilizer.
  /// @throws std::exception if called pre-finalize, see
  /// MultibodyPlant::Finalize().
  bool is_floating() const {
    DRAKE_BODY_THROW_IF_NOT_FINALIZED();
    return topology_.is_floating;
  }

  /// (Advanced) If `true`, this body is a floating body modeled with a
  /// quaternion floating mobilizer. By implication, is_floating() is also
  /// `true`.
  /// @see floating_positions_start(), floating_velocities_start().
  /// @throws std::exception if called pre-finalize, see
  /// MultibodyPlant::Finalize().
  bool has_quaternion_dofs() const {
    DRAKE_BODY_THROW_IF_NOT_FINALIZED();
    return topology_.has_quaternion_dofs;
  }

  /// (Advanced) For floating bodies (see is_floating()) this method returns the
  /// index of the first generalized position in the state vector for a
  /// MultibodyPlant model.
  /// Positions for this body are then contiguous starting at this index.
  /// When a floating body is modeled with a quaternion mobilizer (see
  /// has_quaternion_dofs()), the four consecutive entries in the state starting
  /// at this index correspond to the quaternion that parametrizes this body's
  /// orientation.
  /// @throws std::exception if called pre-finalize, see
  /// MultibodyPlant::Finalize().
  int floating_positions_start() const {
    DRAKE_BODY_THROW_IF_NOT_FINALIZED();
    return topology_.floating_positions_start;
  }

  /// (Advanced) For floating bodies (see is_floating()) this method returns the
  /// index of the first generalized velocity in the state vector for a
  /// MultibodyPlant model.
  /// Velocities for this body are then contiguous starting at this index.
  /// @throws std::exception if called pre-finalize, see
  /// MultibodyPlant::Finalize().
  int floating_velocities_start() const {
    DRAKE_BODY_THROW_IF_NOT_FINALIZED();
    return topology_.floating_velocities_start;
  }

  /// Returns the default mass (not Context dependent) for `this` body.
  /// In general, the mass for a body can be a parameter of the model that can
  /// be retrieved with the method get_mass(). When the mass of a body is a
  /// parameter, the value returned by get_default_mass() is used to initialize
  /// the mass parameter in the context.
  double get_default_mass() const { return default_mass_; }

  /// (Advanced) Returns the mass of this body stored in `context`.
  virtual T get_mass(
      const systems::Context<T> &context)const = 0;

  /// (Advanced) Computes the center of mass `p_BoBcm_B` (or `p_Bcm` for short)
  /// of this body measured from this body's frame origin `Bo` and expressed in
  /// the body frame B.
  virtual const Vector3<T> CalcCenterOfMassInBodyFrame(
      const systems::Context<T>& context) const = 0;

  /// (Advanced) Computes the SpatialInertia `I_BBo_B` of `this` body about its
  /// frame origin `Bo` (not necessarily its center of mass) and expressed in
  /// its body frame `B`.
  /// In general, the spatial inertia of a body is a function of state.
  /// Consider for instance the case of a flexible body for which its spatial
  /// inertia in the body frame depends on the generalized coordinates
  /// describing its state of deformation. As a particular case, the spatial
  /// inertia of a RigidBody in its body frame is constant.
  virtual SpatialInertia<T> CalcSpatialInertiaInBodyFrame(
      const systems::Context<T>& context) const = 0;

  /// Returns the pose `X_WB` of this body B in the world frame W as a function
  /// of the state of the model stored in `context`.
  const math::RigidTransform<T>& EvalPoseInWorld(
      const systems::Context<T>& context) const {
    return this->get_parent_tree().EvalBodyPoseInWorld(context, *this);
  }

  /// Returns the spatial velocity `V_WB` of this body B in the world frame W
  /// as a function of the state of the model stored in `context`.
  const SpatialVelocity<T>& EvalSpatialVelocityInWorld(
      const systems::Context<T>& context) const {
    return this->get_parent_tree().EvalBodySpatialVelocityInWorld(
        context, *this);
  }

  /// Gets the sptatial force on `this` body B from `forces` as F_BBo_W:
  /// applied at body B's origin Bo and expressed in world world frame W.
  const SpatialForce<T>& GetForceInWorld(
      const systems::Context<T>&, const MultibodyForces<T>& forces) const {
    DRAKE_THROW_UNLESS(
        forces.CheckHasRightSizeForModel(this->get_parent_tree()));
    return forces.body_forces()[node_index()];
  }

  /// Adds the spatial force on `this` body B, applied at body B's origin Bo and
  /// expressed in the world frame W into `forces`.
  void AddInForceInWorld(const systems::Context<T>&,
                         const SpatialForce<T>& F_Bo_W,
                         MultibodyForces<T>* forces) const {
    DRAKE_THROW_UNLESS(forces != nullptr);
    DRAKE_THROW_UNLESS(
        forces->CheckHasRightSizeForModel(this->get_parent_tree()));
    forces->mutable_body_forces()[node_index()] += F_Bo_W;
  }

  /// Adds the spatial force on `this` body B, applied at point P and
  /// expressed in a frame E into `forces`.
  /// @param[in] context
  ///   The context containing the current state of the model.
  /// @param[in] p_BP_E
  ///   The position of point P in B, expressed in a frame E.
  /// @param[in] F_Bp_E
  ///   The spatial force to be applied on body B at point P, expressed in
  ///   frame E.
  /// @param[in] frame_E
  ///   The expressed-in frame E.
  /// @param[out] forces
  ///   A multibody forces objects that on output will have `F_Bp_E` added.
  /// @throws std::exception if `forces` is nullptr or if it is not consistent
  /// with the model to which `this` body belongs.
  void AddInForce(
      const systems::Context<T>& context,
      const Vector3<T>& p_BP_E, const SpatialForce<T>& F_Bp_E,
      const Frame<T>& frame_E, MultibodyForces<T>* forces) const {
    DRAKE_THROW_UNLESS(forces != nullptr);
    DRAKE_THROW_UNLESS(
        forces->CheckHasRightSizeForModel(this->get_parent_tree()));
    const math::RotationMatrix<T> R_WE =
        frame_E.CalcRotationMatrixInWorld(context);
    const Vector3<T> p_PB_W = -(R_WE * p_BP_E);
    const SpatialForce<T> F_Bo_W = (R_WE * F_Bp_E).Shift(p_PB_W);
    AddInForceInWorld(context, F_Bo_W, forces);
  }

  /// NVI (Non-Virtual Interface) to DoCloneToScalar() templated on the scalar
  /// type of the new clone to be created. This method is mostly intended to be
  /// called by MultibodyTree::CloneToScalar(). Most users should not call this
  /// clone method directly but rather clone the entire parent MultibodyTree if
  /// needed.
  /// @sa MultibodyTree::CloneToScalar()
  template <typename ToScalar>
  std::unique_ptr<Body<ToScalar>> CloneToScalar(
  const internal::MultibodyTree<ToScalar>& tree_clone) const {
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
      const internal::MultibodyTree<double>& tree_clone) const = 0;

  /// Clones this %Body (templated on T) to a body templated on AutoDiffXd.
  virtual std::unique_ptr<Body<AutoDiffXd>> DoCloneToScalar(
      const internal::MultibodyTree<AutoDiffXd>& tree_clone) const = 0;

  /// Clones this %Body (templated on T) to a body templated on Expression.
  virtual std::unique_ptr<Body<symbolic::Expression>> DoCloneToScalar(
      const internal::MultibodyTree<symbolic::Expression>&) const = 0;

  /// @}

 private:
  // Only friends of BodyAttorney (i.e. MultibodyTree) have access to a selected
  // set of private Body methods.
  friend class internal::BodyAttorney<T>;

  // Helper method for throwing an exception within public methods that should
  // not be called pre-finalize. The invoking method should pass its name so
  // that the error message can include that detail.
  void ThrowIfNotFinalized(const char* source_method) const {
    if (!this->get_parent_tree().topology_is_valid()) {
      throw std::runtime_error(
          "From '" + std::string(source_method) + "'. "
          "The model to which this body belongs must be finalized. See "
          "MultibodyPlant::Finalize().");
    }
  }

  // Implementation for MultibodyElement::DoSetTopology().
  // At MultibodyTree::Finalize() time, each body retrieves its topology
  // from the parent MultibodyTree.
  void DoSetTopology(
      const internal::MultibodyTreeTopology& tree_topology) final {
    topology_ = tree_topology.get_body(this->index());
    body_frame_.SetTopology(tree_topology);
  }

  // MultibodyTree has access to the mutable BodyFrame through BodyAttorney.
  BodyFrame<T>& get_mutable_body_frame() {
    return body_frame_;
  }

  // A string identifying the body in its model.
  // Within a MultibodyPlant model this string is guaranteed to be unique by
  // MultibodyPlant's API.
  std::string name_;

  // Body frame associated with this body.
  BodyFrame<T> body_frame_;

  // In general, the mass of a body can be a constant property of the body or a
  // Parameter of the model. The default mass value is directly reported by
  // get_default_mass() in the former case and used to initialize the mass
  // Parameter in the Context in the latter case.
  double default_mass_{0.0};

  // The internal bookkeeping topology struct used by MultibodyTree.
  internal::BodyTopology topology_;
};

/// @cond
// Undef macros defined at the top of the file. From the GSG:
// "Exporting macros from headers (i.e. defining them in a header without
// #undefing them before the end of the header) is extremely strongly
// discouraged."
#undef DRAKE_BODY_THROW_IF_NOT_FINALIZED
/// @endcond

}  // namespace multibody
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class drake::multibody::BodyFrame)
