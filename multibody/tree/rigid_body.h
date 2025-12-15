#pragma once

#include <memory>
#include <string>

#include "drake/common/default_scalars.h"
#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/drake_deprecated.h"
#include "drake/common/unused.h"
#include "drake/multibody/tree/acceleration_kinematics_cache.h"
#include "drake/multibody/tree/frame.h"
#include "drake/multibody/tree/mobilizer.h"
#include "drake/multibody/tree/multibody_element.h"
#include "drake/multibody/tree/multibody_forces.h"
#include "drake/multibody/tree/multibody_tree_indexes.h"
#include "drake/multibody/tree/multibody_tree_system.h"
#include "drake/multibody/tree/parameter_conversion.h"
#include "drake/multibody/tree/position_kinematics_cache.h"
#include "drake/multibody/tree/scoped_name.h"
#include "drake/multibody/tree/spatial_inertia.h"
#include "drake/multibody/tree/velocity_kinematics_cache.h"
#include "drake/systems/framework/context.h"

namespace drake {
namespace multibody {

// Forward declaration for RigidBodyFrame<T>.
template <typename T>
class RigidBody;

/// A %RigidBodyFrame is a material Frame that serves as the unique reference
/// frame for a RigidBody.
///
/// Each %RigidBody B has a unique body frame for which we use the same symbol
/// B (with meaning clear from context). We represent a body frame by a
/// %RigidBodyFrame object that is created whenever a %RigidBody is constructed
/// and is owned by the %RigidBody. All properties of a %RigidBody are defined
/// with respect to its %RigidBodyFrame, including its mass properties and
/// attachment locations for joints, constraints, actuators, geometry and so on.
/// Run time motion of the body is defined with respect to the motion of its
/// body frame.
///
/// Note that the body frame associated with a rigid body does not necessarily
/// need to be located at its center of mass nor does it need to be aligned with
/// the body's principal axes, although, in practice, it frequently is.
///
/// A %RigidBodyFrame and %RigidBody are tightly coupled concepts; neither makes
/// sense without the other. Therefore, a %RigidBodyFrame instance is
/// constructed in conjunction with its %RigidBody and cannot be constructed
/// anywhere else. However, you can still access the frame associated with a
/// body, see RigidBody::body_frame(). This access is more than a convenience;
/// you can use the %RigidBodyFrame to define other frames on the body and to
/// attach other multibody elements to it.
///
/// @tparam_default_scalar
template <typename T>
class RigidBodyFrame final : public Frame<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(RigidBodyFrame);

  ~RigidBodyFrame() override;

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

  std::unique_ptr<Frame<T>> DoShallowClone() const override;

  math::RigidTransform<T> DoCalcPoseInBodyFrame(
      const systems::Parameters<T>&) const override {
    return math::RigidTransform<T>::Identity();
  }

  math::RotationMatrix<T> DoCalcRotationMatrixInBodyFrame(
      const systems::Parameters<T>&) const override {
    return math::RotationMatrix<T>::Identity();
  }

  math::RigidTransform<T> DoCalcOffsetPoseInBody(
      const systems::Parameters<T>&,
      const math::RigidTransform<T>& X_FQ) const override {
    return X_FQ;
  }

  math::RotationMatrix<T> DoCalcOffsetRotationMatrixInBody(
      const systems::Parameters<T>&,
      const math::RotationMatrix<T>& R_FQ) const override {
    return R_FQ;
  }

 private:
  // RigidBody<T> and RigidBodyFrame<T> are natural allies. A RigidBodyFrame
  // object is created every time a RigidBody object is created and they are
  // associated with each other.
  friend class RigidBody<T>;

  // Make RigidBodyFrame templated on any other scalar type a friend of
  // RigidBodyFrame<T> so that CloneToScalar<ToAnyOtherScalar>() can access
  // private methods from RigidBodyFrame<T>.
  template <typename>
  friend class RigidBodyFrame;

  // Only RigidBody objects can create RigidBodyFrame objects since RigidBody is
  // a friend of RigidBodyFrame.
  explicit RigidBodyFrame(const RigidBody<T>& body)
      : Frame<T>(body.name(), body) {}

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
// private methods in RigidBody. RigidBodyAttorney serves as a "proxy" to the
// RigidBody class but only providing an interface to a selected subset of
// methods that should be accessible to MultibodyTree. These methods are related
// to the construction and finalize stage of the multibody system.
class RigidBodyAttorney {
 private:
  // MultibodyTree keeps a list of mutable pointers to each of the body frames
  // in the system and therefore it needs mutable access.
  // Notice this method is private and therefore users do not have access to it
  // even in the rare event they'd attempt to peek into the "internal::"
  // namespace.
  static RigidBodyFrame<T>& get_mutable_body_frame(RigidBody<T>* body) {
    return body->get_mutable_body_frame();
  }

  // Used by MultibodyTree _during_ Finalize(), after the call to
  // SetTopology() has appropriately registered which bodies are floating base
  // bodies, but before the MultibodyTree::is_finalized() flag has been set.
  static bool is_floating_base_body_pre_finalize(const RigidBody<T>& body) {
    return body.is_floating_base_body_;
  }

  friend class internal::MultibodyTree<T>;
};
}  // namespace internal
/// @endcond

/// The term **rigid body** implies that the deformations of the body under
/// consideration are so small that they have no significant effect on the
/// overall motions of the body and therefore deformations can be neglected.
/// If deformations are neglected, the distance between any two points on the
/// rigid body remains constant at all times. This invariance of the distance
/// between two arbitrary points is often taken as the definition of a rigid
/// body in classical treatments of multibody mechanics [Goldstein 2001].
/// It can be demonstrated that the unconstrained three-dimensional motions of a
/// rigid body can be described by six coordinates and thus it is often said
/// that a free body in space has six **degrees of freedom**. These degrees of
/// freedom obey the Newton-Euler equations of motion. However, within a
/// MultibodyTree, a %RigidBody is *not* free in space; instead, it is assigned
/// a limited number of degrees of freedom (0-6) with respect to its parent
/// body in the multibody tree by its Mobilizer (also called a
/// "tree joint" or "inboard joint"). Additional constraints on permissible
/// motion can be added using Constraint objects to remove more degrees of
/// freedom.
///
/// - [Goldstein 2001] H Goldstein, CP Poole, JL Safko, Classical Mechanics
///                    (3rd Edition), Addison-Wesley, 2001.
///
/// @tparam_default_scalar
template <typename T>
class RigidBody : public MultibodyElement<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(RigidBody);

  /// Constructs a %RigidBody named `body_name` with the given default
  /// SpatialInertia.
  ///
  /// @param[in] body_name
  ///   A name associated with this body.
  /// @param[in] M_BBo_B
  ///   Spatial inertia of this body B about the frame's origin `Bo` and
  ///   expressed in the body frame B. When not provided, defaults to zero.
  /// @note See @ref multibody_spatial_inertia for details on the monogram
  /// notation used for spatial inertia quantities.
  explicit RigidBody(
      const std::string& body_name,
      const SpatialInertia<double>& M_BBo_B = SpatialInertia<double>::Zero());

  /// Constructs a %RigidBody named `body_name` with the given default
  /// SpatialInertia.
  ///
  /// @param[in] body_name
  ///   A name associated with this body.
  /// @param[in] model_instance
  ///   The model instance associated with this body.
  /// @param[in] M_BBo_B
  ///   Spatial inertia of this body B about the frame's origin `Bo` and
  ///   expressed in the body frame B. When not provided, defaults to zero.
  /// @note See @ref multibody_spatial_inertia for details on the monogram
  /// notation used for spatial inertia quantities.
  RigidBody(
      const std::string& body_name, ModelInstanceIndex model_instance,
      const SpatialInertia<double>& M_BBo_B = SpatialInertia<double>::Zero());

  ~RigidBody() override;

  /// Returns this element's unique index.
  BodyIndex index() const { return this->template index_impl<BodyIndex>(); }

  /// Gets the `name` associated with this rigid body. The name will never be
  /// empty.
  const std::string& name() const { return name_; }

  /// Returns scoped name of this body. Neither of the two pieces of the name
  /// will be empty (the scope name and the element name).
  /// @throws std::exception if this element is not associated with a
  ///   MultibodyPlant.
  ScopedName scoped_name() const;

  /// Returns a const reference to the associated BodyFrame.
  const RigidBodyFrame<T>& body_frame() const { return body_frame_; }

  /// For a floating base %RigidBody, lock its inboard joint. Its generalized
  /// velocities will be 0 until it is unlocked.
  /// @throws std::exception if this body is not a floating base body.
  void Lock(systems::Context<T>* context) const {
    ThrowIfNotFinalized(__func__);
    // TODO(rpoyner-tri): consider extending the design to allow locking on
    //  non-floating bodies.
    if (!is_floating_base_body()) {
      // TODO(jwnimmer-tri) This code is not supposed to be inlined (GSG).
      throw std::logic_error(fmt::format(
          "Attempted to call Lock() on non-floating-base rigid body {}",
          name()));
    }
    mobilizer().Lock(context);
  }

  /// For a floating base %RigidBody, unlock its inboard joint.
  /// @throws std::exception if this body is not a floating base body.
  void Unlock(systems::Context<T>* context) const {
    ThrowIfNotFinalized(__func__);
    // TODO(rpoyner-tri): consider extending the design to allow locking on
    //  non-floating bodies.
    if (!is_floating_base_body()) {
      // TODO(jwnimmer-tri) This code is not supposed to be inlined (GSG).
      throw std::logic_error(fmt::format(
          "Attempted to call Unlock() on non-floating-base rigid body {}",
          name()));
    }
    mobilizer().Unlock(context);
  }

  /// Determines whether this %RigidBody is currently locked to its inboard
  /// (parent) %RigidBody. This is not limited to floating base bodies but
  /// generally Joint::is_locked() is preferable otherwise.
  /// @returns true if the body is locked, false otherwise.
  bool is_locked(const systems::Context<T>& context) const {
    ThrowIfNotFinalized(__func__);
    DRAKE_ASSERT(this->has_parent_tree());
    return mobilizer().is_locked(context);
  }

  /// (Advanced) Returns the index of the mobilized body ("mobod") in the
  /// computational directed forest structure of the owning MultibodyTree to
  /// which this %RigidBody belongs. This serves as the BodyNode index and the
  /// index into all associated quantities.
  internal::MobodIndex mobod_index() const { return mobilizer().index(); }

  /// (Advanced) Returns `true` if this body is a _floating base body_, meaning
  /// it had no explicit joint to a parent body so is mobilized by an
  /// automatically-added (ephemeral) floating (6 dof) joint to World.
  ///
  /// @note A floating base body is not necessarily modeled with a quaternion
  /// mobilizer, see has_quaternion_dofs(). Alternative options include a
  /// roll-pitch-yaw (rpy) parametrization of rotations, see
  /// RpyFloatingMobilizer.
  ///
  /// @throws std::exception if called pre-finalize,
  /// @see MultibodyPlant::Finalize()
  bool is_floating_base_body() const {
    ThrowIfNotFinalized(__func__);
    return is_floating_base_body_;
  }

  DRAKE_DEPRECATED("2026-06-01", "Use is_floating_base_body() instead.")
  bool is_floating() const { return is_floating_base_body(); }

  /// (Advanced) If `true`, this body's generalized position coordinates q
  /// include a quaternion, which occupies the first four elements of q. Note
  /// that this does not imply that the body is floating base body since it may
  /// have fewer than 6 dofs or its inboard body could be something other than
  /// World.
  /// @throws std::exception if called pre-finalize
  /// @see is_floating_base_body(), MultibodyPlant::Finalize()
  bool has_quaternion_dofs() const {
    ThrowIfNotFinalized(__func__);
    return mobilizer().has_quaternion_dofs();
  }

  /// (Advanced) For floating base bodies (see is_floating_base_body()),
  /// returns the index of this %RigidBody's first generalized position in the
  /// vector q of generalized position coordinates for a MultibodyPlant model.
  /// Positions q for this %RigidBody are then contiguous starting at this
  /// index. When a floating %RigidBody is modeled with quaternion coordinates
  /// (see has_quaternion_dofs()), the four consecutive entries in the state
  /// starting at this index correspond to the quaternion that parametrizes this
  /// %RigidBody's orientation.
  /// @throws std::exception if called pre-finalize
  /// @pre this is a floating base body
  /// @see is_floating_base_body(), has_quaternion_dofs()
  /// @see MultibodyPlant::Finalize()
  int floating_positions_start() const {
    ThrowIfNotFinalized(__func__);
    DRAKE_DEMAND(is_floating_base_body());
    return mobilizer().position_start_in_q();
  }

  /// (Advanced) For floating base bodies (see is_floating_base_body()),
  /// returns the index of this %RigidBody's first generalized velocity in the
  /// vector v of generalized velocities for a MultibodyPlant model. Velocities
  /// v for this %RigidBody are then contiguous starting at this index.
  /// @throws std::exception if called pre-finalize
  /// @pre this is a floating base body
  /// @see is_floating_base_body(), MultibodyPlant::Finalize()
  int floating_velocities_start_in_v() const {
    ThrowIfNotFinalized(__func__);
    DRAKE_DEMAND(is_floating_base_body());
    return mobilizer().velocity_start_in_v();
  }

  /// Returns a string suffix (e.g. to be appended to the name()) to identify
  /// the `k`th position in the floating base body. `position_index_in_body`
  /// must be in [0, 7) if `has_quaternion_dofs()` is true, otherwise in [0, 6).
  /// @throws std::exception if called pre-finalize
  /// @pre this is a floating base body
  /// @see is_floating_base_body(), has_quaternion_dofs()
  /// @see MultibodyPlant::Finalize()
  std::string floating_position_suffix(int position_index_in_body) const {
    ThrowIfNotFinalized(__func__);
    DRAKE_DEMAND(is_floating_base_body());
    if (has_quaternion_dofs()) {
      DRAKE_DEMAND(0 <= position_index_in_body && position_index_in_body < 7);
    } else {
      DRAKE_DEMAND(0 <= position_index_in_body && position_index_in_body < 6);
    }
    return mobilizer().position_suffix(position_index_in_body);
  }

  /// Returns a string suffix (e.g. to be appended to the name()) to identify
  /// the `k`th velocity in the floating base body. `velocity_index_in_body`
  /// must be in [0,6).
  /// @throws std::exception if called pre-finalize
  /// @pre this is a floating base body
  /// @see is_floating_base_body(), MultibodyPlant::Finalize()
  std::string floating_velocity_suffix(int velocity_index_in_body) const {
    ThrowIfNotFinalized(__func__);
    DRAKE_DEMAND(is_floating_base_body());
    DRAKE_DEMAND(0 <= velocity_index_in_body && velocity_index_in_body < 6);
    DRAKE_ASSERT(this->has_parent_tree());
    return mobilizer().velocity_suffix(velocity_index_in_body);
  }

  /// Returns this %RigidBody's default mass, which is initially supplied at
  /// construction when specifying this body's SpatialInertia.
  /// @note In general, a rigid body's mass can be a constant property stored in
  /// this rigid body's %SpatialInertia or a parameter that is stored in a
  /// Context. The default constant mass value is used to initialize the mass
  /// parameter in the Context.
  double default_mass() const { return default_spatial_inertia_.get_mass(); }

  /// Returns the default value of this %RigidBody's center of mass as measured
  /// and expressed in its body frame. This value is initially supplied at
  /// construction when specifying this body's SpatialInertia.
  /// @retval p_BoBcm_B The position of this rigid body B's center of mass `Bcm`
  /// measured from Bo (B's frame origin) and expressed in B (body B's frame).
  const Vector3<double>& default_com() const {
    return default_spatial_inertia_.get_com();
  }

  /// Returns the default value of this body B's unit inertia about Bo (body B's
  /// origin), expressed in B (this body's body frame). This value is initially
  /// supplied at construction when specifying this body's SpatialInertia.
  /// @retval G_BBo_B rigid body B's unit inertia about Bo, expressed in B.
  const UnitInertia<double>& default_unit_inertia() const {
    return default_spatial_inertia_.get_unit_inertia();
  }

  /// Gets the default value of this body B's rotational inertia about Bo
  /// (B's origin), expressed in B (this body's body frame). This value is
  /// calculated from the SpatialInertia supplied at construction of this body.
  /// @retval I_BBo_B body B's rotational inertia about Bo, expressed in B.
  RotationalInertia<double> default_rotational_inertia() const {
    return default_spatial_inertia_.CalcRotationalInertia();
  }

  /// Gets the default value of this body B's SpatialInertia about Bo
  /// (B's origin) and expressed in B (this body's frame).
  /// @retval M_BBo_B body B's spatial inertia about Bo, expressed in B.
  const SpatialInertia<double>& default_spatial_inertia() const {
    return default_spatial_inertia_;
  }

  /// Gets this body's mass from the given context.
  /// @param[in] context contains the state of the multibody system.
  /// @pre the context makes sense for use by this RigidBody.
  const T& get_mass(const systems::Context<T>& context) const {
    const systems::BasicVector<T>& spatial_inertia_parameter =
        context.get_numeric_parameter(spatial_inertia_parameter_index_);
    return internal::parameter_conversion::GetMass(spatial_inertia_parameter);
  }

  /// Returns the pose `X_WB` of this %RigidBody B in the world frame W as a
  /// function of the state of the model stored in `context`.
  const math::RigidTransform<T>& EvalPoseInWorld(
      const systems::Context<T>& context) const {
    ThrowIfNotFinalized(__func__);
    DRAKE_ASSERT(this->has_parent_tree());
    return this->get_parent_tree().EvalBodyPoseInWorld(context, *this);
  }

  /// Evaluates V_WB, this body B's SpatialVelocity in the world frame W.
  /// @param[in] context Contains the state of the model.
  /// @retval V_WB_W this body B's spatial velocity in the world frame W,
  /// expressed in W (for point Bo, the body frame's origin).
  const SpatialVelocity<T>& EvalSpatialVelocityInWorld(
      const systems::Context<T>& context) const {
    ThrowIfNotFinalized(__func__);
    DRAKE_ASSERT(this->has_parent_tree());
    return this->get_parent_tree().EvalBodySpatialVelocityInWorld(context,
                                                                  *this);
  }

  /// Evaluates A_WB, this body B's SpatialAcceleration in the world frame W.
  /// @param[in] context Contains the state of the model.
  /// @retval A_WB_W this body B's spatial acceleration in the world frame W,
  /// expressed in W (for point Bo, the body's origin).
  /// @note When cached values are out of sync with the state stored in context,
  /// this method performs an expensive forward dynamics computation, whereas
  /// once evaluated, successive calls to this method are inexpensive.
  const SpatialAcceleration<T>& EvalSpatialAccelerationInWorld(
      const systems::Context<T>& context) const {
    ThrowIfNotFinalized(__func__);
    DRAKE_ASSERT(this->has_parent_tree());
    return this->get_parent_tree().EvalBodySpatialAccelerationInWorld(context,
                                                                      *this);
  }

  /// Gets the SpatialForce on this %RigidBody B from `forces` as F_BBo_W:
  /// applied at body B's origin Bo and expressed in world frame W.
  const SpatialForce<T>& GetForceInWorld(
      const systems::Context<T>&, const MultibodyForces<T>& forces) const {
    ThrowIfNotFinalized(__func__);
    DRAKE_ASSERT(this->has_parent_tree());
    DRAKE_THROW_UNLESS(
        forces.CheckHasRightSizeForModel(this->get_parent_tree()));
    return forces.body_forces()[mobod_index()];
  }

  /// Adds the SpatialForce on this %RigidBody B, applied at body B's origin Bo
  /// and expressed in the world frame W into `forces`.
  void AddInForceInWorld(const systems::Context<T>&,
                         const SpatialForce<T>& F_Bo_W,
                         MultibodyForces<T>* forces) const {
    DRAKE_THROW_UNLESS(forces != nullptr);
    ThrowIfNotFinalized(__func__);
    DRAKE_ASSERT(this->has_parent_tree());
    DRAKE_THROW_UNLESS(
        forces->CheckHasRightSizeForModel(this->get_parent_tree()));
    forces->mutable_body_forces()[mobod_index()] += F_Bo_W;
  }

  /// Adds the SpatialForce on this %RigidBody B, applied at point P and
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
  /// with the model to which this body belongs.
  void AddInForce(const systems::Context<T>& context, const Vector3<T>& p_BP_E,
                  const SpatialForce<T>& F_Bp_E, const Frame<T>& frame_E,
                  MultibodyForces<T>* forces) const;

  /// Gets this body's center of mass position from the given context.
  /// @param[in] context contains the state of the multibody system.
  /// @returns p_BoBcm_B position vector from Bo (this rigid body B's origin)
  /// to Bcm (B's center of mass), expressed in B.
  /// @pre the context makes sense for use by this %RigidBody.
  Vector3<T> CalcCenterOfMassInBodyFrame(
      const systems::Context<T>& context) const {
    const systems::BasicVector<T>& spatial_inertia_parameter =
        context.get_numeric_parameter(spatial_inertia_parameter_index_);
    return internal::parameter_conversion::GetCenterOfMass(
        spatial_inertia_parameter);
  }

  /// Calculates Bcm's translational velocity in the world frame W.
  /// @param[in] context The context contains the state of the model.
  /// @retval v_WBcm_W The translational velocity of Bcm (this rigid body's
  /// center of mass) in the world frame W, expressed in W.
  Vector3<T> CalcCenterOfMassTranslationalVelocityInWorld(
      const systems::Context<T>& context) const;

  /// Calculates Bcm's translational acceleration in the world frame W.
  /// @param[in] context The context contains the state of the model.
  /// @retval a_WBcm_W The translational acceleration of Bcm (this rigid body's
  /// center of mass) in the world frame W, expressed in W.
  /// @note When cached values are out of sync with the state stored in context,
  /// this method performs an expensive forward dynamics computation, whereas
  /// once evaluated, successive calls to this method are inexpensive.
  Vector3<T> CalcCenterOfMassTranslationalAccelerationInWorld(
      const systems::Context<T>& context) const;

  /// Gets this body's spatial inertia about its origin from the given context.
  /// @param[in] context contains the state of the multibody system.
  /// @returns M_BBo_B spatial inertia of this rigid body B about Bo (B's
  /// origin), expressed in B. M_BBo_B contains properties related to B's mass,
  /// the position vector from Bo to Bcm (B's center of mass), and G_BBo_B
  /// (B's unit inertia about Bo expressed in B).
  /// @pre the context makes sense for use by this %RigidBody.
  SpatialInertia<T> CalcSpatialInertiaInBodyFrame(
      const systems::Context<T>& context) const {
    // TODO(joemasterjohn): Speed this up when we can store a reference to a
    //  SpatialInertia<T> as an abstract parameter.
    const systems::BasicVector<T>& spatial_inertia_parameter =
        context.get_numeric_parameter(spatial_inertia_parameter_index_);
    return internal::parameter_conversion::ToSpatialInertia(
        spatial_inertia_parameter);
  }

  /// For this %RigidBody B, sets its mass stored in @p context to @p mass.
  /// @param[out] context contains the state of the multibody system.
  /// @param[in] mass mass of this rigid body B.
  /// @note This function changes this body B's mass and appropriately scales
  /// I_BBo_B (B's rotational inertia about Bo, expressed in B).
  /// @pre the context makes sense for use by this RigidBody.
  /// @throws std::exception if context is null.
  void SetMass(systems::Context<T>* context, const T& mass) const {
    DRAKE_THROW_UNLESS(context != nullptr);
    systems::BasicVector<T>& spatial_inertia_parameter =
        context->get_mutable_numeric_parameter(
            spatial_inertia_parameter_index_);
    spatial_inertia_parameter.SetAtIndex(
        internal::parameter_conversion::SpatialInertiaIndex::k_mass, mass);
  }

  /// (Advanced) Sets this body's center of mass position while preserving its
  /// inertia about its body origin.
  /// @param[in, out] context contains the state of the multibody system. It is
  /// modified to store the updated com (center of mass position).
  /// @param[in] com position vector from Bo (this body B's origin) to Bcm
  /// (B's center of mass), expressed in B.
  /// @note This function changes B's center of mass position **without**
  /// modifying G_BBo_B (B's unit inertia about Bo, expressed in B). Since this
  /// use case is very unlikely, consider using SetSpatialInertiaInBodyFrame()
  /// or SetCenterOfMassInBodyFrameAndPreserveCentralInertia().
  /// @pre the context makes sense for use by this %RigidBody.
  /// @throws std::exception if context is null.
  /// @warning Do not use this function unless it is needed (think twice).
  // TODO(Mitiguy) Consider deprecating this function.
  void SetCenterOfMassInBodyFrame(systems::Context<T>* context,
                                  const Vector3<T>& com) const {
    SetCenterOfMassInBodyFrameNoModifyInertia(context, com);
  }

  /// Sets this body's center of mass position while preserving its inertia
  /// about its center of mass.
  /// @param[in, out] context contains the state of the multibody system. It is
  /// modified to store the updated center_of_mass_position and the updated
  /// G_BBo_B (this body B's unit inertia about B's origin Bo, expressed in B).
  /// @param[in] center_of_mass_position position vector from Bo to Bcm
  /// (B's center of mass), expressed in B.
  /// @note G_BBo_B is modified to ensure B's inertia about Bcm is unchanged.
  /// Although this function can work well when B's mass is concentrated at (or
  /// mostly near) a single point, it has **questionable** utility to generally
  /// account for inertia changes due to arbitrary center of mass changes.
  /// Consider using SetSpatialInertiaInBodyFrame() instead.
  /// @pre the context makes sense for use by this RigidBody.
  /// @throws std::exception if context is null.
  void SetCenterOfMassInBodyFrameAndPreserveCentralInertia(
      systems::Context<T>* context,
      const Vector3<T>& center_of_mass_position) const;

  /// For this %RigidBody B, sets its SpatialInertia that is stored in
  /// @p context to @p M_Bo_B.
  /// @param[out] context contains the state of the multibody system.
  /// @param[in] M_Bo_B spatial inertia of this rigid body B about Bo (B's
  /// origin), expressed in B. M_Bo_B contains properties related to B's mass,
  /// the position vector from Bo to Bcm (B's center of mass), and G_Bo_B
  /// (B's unit inertia about Bo expressed in B).
  /// @pre the context makes sense for use by this %RigidBody.
  /// @throws std::exception if context is null.
  void SetSpatialInertiaInBodyFrame(systems::Context<T>* context,
                                    const SpatialInertia<T>& M_Bo_B) const {
    DRAKE_THROW_UNLESS(context != nullptr);
    systems::BasicVector<T>& spatial_inertia_parameter =
        context->get_mutable_numeric_parameter(
            spatial_inertia_parameter_index_);
    spatial_inertia_parameter.SetFrom(
        internal::parameter_conversion::ToBasicVector(M_Bo_B));
  }

  /// @name Methods to access position kinematics quantities.
  /// The input PositionKinematicsCache to these methods must be in sync with
  /// context.  These method's APIs will be deprecated when caching arrives.
  ///@{

  /// (Advanced) Extract this body's pose in world (from the position
  /// kinematics).
  /// @param[in] pc position kinematics cache.
  /// @retval X_WB pose of rigid body B in world frame W.
  const math::RigidTransform<T>& get_pose_in_world(
      const internal::PositionKinematicsCache<T>& pc) const {
    return pc.get_X_WB(this->mobod_index());
  }

  /// (Advanced) Extract the RotationMatrix relating the world frame to this
  /// body's frame.
  /// @param[in] pc position kinematics cache.
  /// @retval R_WB rotation matrix relating rigid body B in world frame W.
  const math::RotationMatrix<T> get_rotation_matrix_in_world(
      const internal::PositionKinematicsCache<T>& pc) const {
    return get_pose_in_world(pc).rotation();
  }

  /// (Advanced) Extract the position vector from world origin to this body's
  /// origin, expressed in world.
  /// @param[in] pc position kinematics cache.
  /// @retval p_WoBo_W position vector from Wo (world origin) to
  ///         Bo (this body's origin) expressed in W (world).
  const Vector3<T> get_origin_position_in_world(
      const internal::PositionKinematicsCache<T>& pc) const {
    return get_pose_in_world(pc).translation();
  }
  ///@}

  /// @name Methods to access velocity kinematics quantities.
  /// The input VelocityKinematicsCache to these methods must be in sync with
  /// context.  These method's APIs will be deprecated when caching arrives.
  ///@{

  /// (Advanced) Returns V_WB, this %RigidBody B's SpatialVelocity in
  /// the world frame W.
  /// @param[in] vc velocity kinematics cache.
  /// @retval V_WB_W this rigid body B's spatial velocity in the world
  /// frame W, expressed in W (for point Bo, the body frame's origin).
  const SpatialVelocity<T>& get_spatial_velocity_in_world(
      const internal::VelocityKinematicsCache<T>& vc) const {
    return vc.get_V_WB(this->mobod_index());
  }

  /// (Advanced) Extract this body's angular velocity in world, expressed in
  /// world.
  /// @param[in] vc velocity kinematics cache.
  /// @retval w_WB_W rigid body B's angular velocity in world W, expressed in W.
  const Vector3<T>& get_angular_velocity_in_world(
      const internal::VelocityKinematicsCache<T>& vc) const {
    return get_spatial_velocity_in_world(vc).rotational();
  }

  /// (Advanced) Extract the velocity of this body's origin in world, expressed
  /// in world.
  /// @param[in] vc velocity kinematics cache.
  /// @retval v_WBo_W velocity of Bo (body origin) in world W, expressed in W.
  const Vector3<T>& get_origin_velocity_in_world(
      const internal::VelocityKinematicsCache<T>& vc) const {
    return get_spatial_velocity_in_world(vc).translational();
  }
  ///@}

  /// @name Methods to access acceleration kinematics quantities.
  /// The input AccelerationKinematicsCache to these methods must be in sync
  /// with context.  These method APIs will be deprecated when caching arrives.
  ///@{

  /// (Advanced) Returns A_WB, this %RigidBody B's SpatialAcceleration in
  /// the world frame W.
  /// @param[in] ac acceleration kinematics cache.
  /// @retval A_WB_W this rigid body B's spatial acceleration in the world
  /// frame W, expressed in W (for point Bo, the body frame's origin).
  const SpatialAcceleration<T>& get_spatial_acceleration_in_world(
      const internal::AccelerationKinematicsCache<T>& ac) const {
    return ac.get_A_WB(this->mobod_index());
  }

  /// (Advanced) Extract this body's angular acceleration in world, expressed
  /// in world.
  /// @param[in] ac velocity kinematics cache.
  /// @retval alpha_WB_W B's angular acceleration in world W, expressed in W.
  const Vector3<T>& get_angular_acceleration_in_world(
      const internal::AccelerationKinematicsCache<T>& ac) const {
    return get_spatial_acceleration_in_world(ac).rotational();
  }

  /// (Advanced) Extract acceleration of this body's origin in world, expressed
  /// in world.
  /// @param[in] ac acceleration kinematics cache.
  /// @retval a_WBo_W acceleration of body origin Bo in world W, expressed in W.
  const Vector3<T>& get_origin_acceleration_in_world(
      const internal::AccelerationKinematicsCache<T>& ac) const {
    return get_spatial_acceleration_in_world(ac).translational();
  }
  ///@}

  /// (Advanced) This method is mostly intended to be called by
  /// MultibodyTree::CloneToScalar(). Most users should not call this clone
  /// method directly but rather clone the entire parent MultibodyTree if
  /// needed.
  /// @sa MultibodyTree::CloneToScalar()
  template <typename ToScalar>
  std::unique_ptr<RigidBody<ToScalar>> CloneToScalar(
      const internal::MultibodyTree<ToScalar>& tree_clone) const {
    return TemplatedDoCloneToScalar(tree_clone);
  }

 private:
  // Only friends of RigidBodyAttorney (i.e. MultibodyTree) have access to a
  // selected set of private RigidBody methods.
  friend class internal::RigidBodyAttorney<T>;

  // Called near the end of Finalize().
  void DoSetTopology() final {
    DRAKE_DEMAND(mobilizer_ == nullptr);
    const internal::MultibodyTree<T>& tree = this->get_parent_tree();
    const internal::SpanningForest& forest = tree.forest();
    const internal::LinkJointGraph::Link& link = forest.link_by_index(index());
    mobilizer_ = &tree.get_mobilizer(link.mobod_index());

    // Is this RigidBody the active link on its Mobod?
    const bool is_active_link =
        link.ordinal() == forest.mobods(link.mobod_index()).link_ordinal();
    is_floating_base_body_ =
        is_active_link && mobilizer_->is_floating_base_mobilizer();
  }

  // Implementation for MultibodyElement::DoDeclareParameters().
  void DoDeclareParameters(
      internal::MultibodyTreeSystem<T>* tree_system) final {
    // Sets model values to dummy values to indicate that the model values are
    // not used. This class stores the the default values of the parameters.
    // 10 numeric values are used to store mass, center of mass, moments and
    // products of inertia packed into one basic vector.
    spatial_inertia_parameter_index_ =
        this->DeclareNumericParameter(tree_system, systems::BasicVector<T>(10));
  }

  // Implementation for MultibodyElement::DoSetDefaultParameters().
  void DoSetDefaultParameters(systems::Parameters<T>* parameters) const final {
    // Set the default spatial inertia.
    systems::BasicVector<T>& spatial_inertia_parameter =
        parameters->get_mutable_numeric_parameter(
            spatial_inertia_parameter_index_);
    spatial_inertia_parameter.SetFrom(
        internal::parameter_conversion::ToBasicVector<T>(
            default_spatial_inertia_.template cast<T>()));
  }

  // Helper method for throwing an exception within public methods that should
  // not be called pre-finalize. The invoking method should pass its name so
  // that the error message can include that detail.
  void ThrowIfNotFinalized(const char* source_method) const {
    DRAKE_THROW_UNLESS(this->has_parent_tree());
    if (!this->get_parent_tree().is_finalized()) {
      // TODO(jwnimmer-tri) This code is not supposed to be inlined (GSG).
      throw std::runtime_error(
          "From '" + std::string(source_method) +
          "'. The model to which this rigid body belongs must be finalized. "
          "See MultibodyPlant::Finalize().");
    }
  }

  // For this RigidBody B, set its center of mass position stored in context
  // to center_of_mass_position, but does not modify other inertia properties.
  // @param[in, out] context contains the state of the multibody system.
  // @param[in] center_of_mass_position position vector from Bo (B's origin) to
  // Bcm (B's center of mass), expressed in B.
  // @note G_BBo_B and I_BBo_B (B's unit inertia and rotational inertia about
  // Bo, expressed in B) are **not** changed. In general, this means G_BBcm_B
  // and I_BBcm_B **are** changed. To avoid invalid inertia properties, consider
  // if changing center of mass position also changes G_BBo_B and necessitates
  // a call to SetUnitInertiaAboutBodyOrigin(). B's inertia properties can be
  // checked via CalcSpatialInertiaInBodyFrame().IsPhysicallyValid().
  // @pre the context makes sense for use by this %RigidBody.
  // @throws std::exception if context is null.
  void SetCenterOfMassInBodyFrameNoModifyInertia(
      systems::Context<T>* context,
      const Vector3<T>& center_of_mass_position) const;

  // For this RigidBody B, sets the unit inertia about B's origin stored in
  // @p context to @p G_BBo_B.
  // @param[in, out] context contains the state of the multibody system.
  // @param[in] G_BBo_B B's unit inertia about Bo (B's origin), expressed in B.
  // @note To avoid invalid inertia properties, consider if changing G_BBo_B
  // also changes B's center of mass and necessitates a call to
  // SetCenterOfMassInBodyFrameNoModifyInertia(). B's inertia properties can be
  // checked via CalcSpatialInertiaInBodyFrame().IsPhysicallyValid().
  // @pre the context makes sense for use by this %RigidBody.
  // @throws std::exception if context is null.
  void SetUnitInertiaAboutBodyOrigin(systems::Context<T>* context,
                                     const UnitInertia<T>& G_BBo_B) const;

  // Helper method to make a clone templated on ToScalar.
  template <typename ToScalar>
  std::unique_ptr<RigidBody<ToScalar>> TemplatedDoCloneToScalar(
      const internal::MultibodyTree<ToScalar>& tree_clone) const {
    unused(tree_clone);
    return std::make_unique<RigidBody<ToScalar>>(this->name(),
                                                 default_spatial_inertia_);
  }

  // MultibodyTree has access to the mutable RigidBodyFrame through
  // RigidBodyAttorney.
  RigidBodyFrame<T>& get_mutable_body_frame() { return body_frame_; }

  const internal::Mobilizer<T>& mobilizer() const {
    DRAKE_ASSERT(mobilizer_ != nullptr);
    return *mobilizer_;
  }

  // A string identifying the body in its model.
  // Within a MultibodyPlant model instance this string is guaranteed to be
  // unique by MultibodyPlant's API.
  const std::string name_;

  // Body frame associated with this body.
  RigidBodyFrame<T> body_frame_;

  // Spatial inertia about the body frame origin Bo, expressed in B.
  SpatialInertia<double> default_spatial_inertia_;

  // System parameter index for this bodies SpatialInertia stored in a
  // context.
  systems::NumericParameterIndex spatial_inertia_parameter_index_;

  // Below here, members are set at Finalize() via SetTopology().

  // The mobilizer of the Mobod that this body follows.
  const internal::Mobilizer<T>* mobilizer_{};

  // True if the mobilizer is a floating base mobilizer and this body is the
  // active link of the mobilized composite.
  bool is_floating_base_body_{false};
};

/// (Compatibility) Prefer RigidBody to Body, however this dispreferred alias
/// is available to permit older code to continue working.
template <typename T>
using Body = RigidBody<T>;

}  // namespace multibody
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::multibody::RigidBodyFrame);

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::multibody::RigidBody);
