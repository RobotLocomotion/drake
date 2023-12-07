#pragma once

#include <memory>
#include <string>
#include <vector>

#include "drake/common/default_scalars.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/drake_deprecated.h"
#include "drake/common/unused.h"
#include "drake/multibody/tree/frame.h"
#include "drake/multibody/tree/multibody_element.h"
#include "drake/multibody/tree/multibody_forces.h"
#include "drake/multibody/tree/multibody_tree_indexes.h"
#include "drake/multibody/tree/multibody_tree_topology.h"
#include "drake/multibody/tree/scoped_name.h"
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
/// Each Body B, has a unique body frame for which we use the same symbol B
/// (with meaning clear from context). All properties of a body are defined with
/// respect to its body frame, including its mass properties and attachment
/// locations for joints, constraints, actuators, geometry and so on. Run time
/// motion of the body is defined with respect to the motion of its body frame.
/// We represent a body frame by a %BodyFrame object that is created whenever a
/// Body is constructed and is owned by the Body.
///
/// Note that the %BodyFrame associated with a body does not necessarily need to
/// be located at its center of mass nor does it need to be aligned with the
/// body's principal axes, although, in practice, it frequently is.
///
/// A %BodyFrame and Body are tightly coupled concepts; neither makes sense
/// without the other. Therefore, a %BodyFrame instance is constructed in
/// conjunction with its Body and cannot be constructed anywhere else. However,
/// you can still access the frame associated with a body, see
/// Body::body_frame(). This access is more than a convenience; you can use the
/// %BodyFrame to define other frames on the body and to attach other multibody
/// elements to it.
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
/// As an element or component of a MultibodyPlant, a body is a
/// MultibodyElement, and therefore it has a unique index of type BodyIndex
/// within the multibody plant it belongs to.
///
/// A %Body contains a unique BodyFrame; see BodyFrame class documentation for
/// more information.
///
/// @tparam_default_scalar
template <typename T>
class Body : public MultibodyElement<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(Body)

  /// Returns this element's unique index.
  BodyIndex index() const { return this->template index_impl<BodyIndex>(); }

  /// Gets the `name` associated with `this` body. The name will never be empty.
  const std::string& name() const { return name_; }

  /// Returns scoped name of this frame. Neither of the two pieces of the name
  /// will be empty (the scope name and the element name).
  ScopedName scoped_name() const;

  /// Returns a const reference to the associated BodyFrame.
  const BodyFrame<T>& body_frame() const {
    return body_frame_;
  }

  /// For a floating body, lock its inboard joint. Its generalized
  /// velocities will be 0 until it is unlocked.
  /// @throws std::exception if this body is not a floating body.
  void Lock(systems::Context<T>* context) const {
    // TODO(rpoyner-tri): consider extending the design to allow locking on
    //  non-floating bodies.
    if (!is_floating()) {
      throw std::logic_error(fmt::format(
          "Attempted to call Lock() on non-floating body {}", name()));
    }
    this->get_parent_tree()
        .get_mobilizer(topology_.inboard_mobilizer)
        .Lock(context);
  }

  /// For a floating body, unlock its inboard joint.
  /// @throws std::exception if this body is not a floating body.
  void Unlock(systems::Context<T>* context) const {
    // TODO(rpoyner-tri): consider extending the design to allow locking on
    //  non-floating bodies.
    if (!is_floating()) {
      throw std::logic_error(fmt::format(
          "Attempted to call Unlock() on non-floating body {}", name()));
    }
    this->get_parent_tree()
        .get_mobilizer(topology_.inboard_mobilizer)
        .Unlock(context);
  }

  /// Determines whether this %Body is currently locked to its inboard (parent)
  /// %Body. This is not limited to floating bodies but generally
  /// Joint::is_locked() is preferable otherwise.
  /// @returns true if the body is locked, false otherwise.
  bool is_locked(const systems::Context<T>& context) const {
    return this->get_parent_tree()
        .get_mobilizer(topology_.inboard_mobilizer)
        .is_locked(context);
  }

  /// (Advanced) Returns the index of the mobilized body ("mobod") in the
  /// computational directed forest structure of the owning MultibodyTree to
  /// which this %Body belongs. This serves as the BodyNode index and the index
  /// into all associated quantities.
  internal::MobodIndex mobod_index() const {
    return topology_.mobod_index;
  }

  DRAKE_DEPRECATED("2024-03-01",  "Use mobod_index() instead.")
  internal::MobodIndex node_index() const { return mobod_index(); }

  /// (Advanced) Returns `true` if `this` body is granted 6-dofs by a Mobilizer
  /// and the parent body of this body's associated 6-dof joint is `world`.
  /// @note A floating body is not necessarily modeled with a quaternion
  /// mobilizer, see has_quaternion_dofs(). Alternative options include a space
  /// XYZ parametrization of rotations, see SpaceXYZMobilizer.
  /// @throws std::exception if called pre-finalize,
  /// @see MultibodyPlant::Finalize()
  bool is_floating() const {
    DRAKE_BODY_THROW_IF_NOT_FINALIZED();
    return topology_.is_floating;
  }

  /// (Advanced) If `true`, this body's generalized position coordinates q
  /// include a quaternion, which occupies the first four elements of q. Note
  /// that this does not imply that the body is floating since it may have
  /// fewer than 6 dofs or its inboard body could be something other than World.
  /// @throws std::exception if called pre-finalize
  /// @see is_floating(), MultibodyPlant::Finalize()
  bool has_quaternion_dofs() const {
    DRAKE_BODY_THROW_IF_NOT_FINALIZED();
    return topology_.has_quaternion_dofs;
  }

  /// (Advanced) For floating bodies (see is_floating()) this method returns the
  /// index of this %Body's first generalized position in the vector q of
  /// generalized position coordinates for a MultibodyPlant model.
  /// Positions q for this %Body are then contiguous starting at this index.
  /// When a floating %Body is modeled with quaternion coordinates (see
  /// has_quaternion_dofs()), the four consecutive entries in the state starting
  /// at this index correspond to the quaternion that parametrizes this %Body's
  /// orientation.
  /// @throws std::exception if called pre-finalize
  /// @pre `this` is a floating body
  /// @see is_floating(), has_quaternion_dofs(), MultibodyPlant::Finalize()
  int floating_positions_start() const {
    DRAKE_BODY_THROW_IF_NOT_FINALIZED();
    DRAKE_DEMAND(is_floating());
    return topology_.floating_positions_start;
  }

  /// (Advanced, Deprecated) For floating bodies (see is_floating()) this
  /// method returns the index of this %Body's first generalized velocity in the
  /// _full state vector_ for a MultibodyPlant model, under the dubious
  /// assumption that the state consists of [q v] concatenated.
  /// Velocities for this body are then contiguous starting at this index.
  /// @throws std::exception if called pre-finalize
  /// @pre `this` is a floating body
  /// @see floating_velocities_start_in_v()
  DRAKE_DEPRECATED("2024-02-01",
                   "Convert to floating_velocities_start_in_v(). In a state "
                   "with [q v] concatenated, offset by num_positions() to "
                   "get to the start of v.")
  int floating_velocities_start() const {
    DRAKE_BODY_THROW_IF_NOT_FINALIZED();
    DRAKE_DEMAND(is_floating());
    return this->get_parent_tree().num_positions() +
           topology_.floating_velocities_start_in_v;
  }

  /// (Advanced) For floating bodies (see is_floating()) this method returns the
  /// index of this %Body's first generalized velocity in the vector v of
  /// generalized velocities for a MultibodyPlant model.
  /// Velocities v for this %Body are then contiguous starting at this index.
  /// @throws std::exception if called pre-finalize
  /// @pre `this` is a floating body
  /// @see is_floating(), MultibodyPlant::Finalize()
  int floating_velocities_start_in_v() const {
    DRAKE_BODY_THROW_IF_NOT_FINALIZED();
    DRAKE_DEMAND(is_floating());
    return topology_.floating_velocities_start_in_v;
  }

  /// Returns a string suffix (e.g. to be appended to the name()) to identify
  /// the `k`th position in the floating base. @p position_index_in_body must
  /// be in [0, 7) if `has_quaternion_dofs()` is true, otherwise in [0, 6).
  /// @throws std::exception if called pre-finalize
  /// @pre `this` is a floating body
  /// @see is_floating(), has_quaternion_dofs(), MultibodyPlant::Finalize()
  std::string floating_position_suffix(int position_index_in_body) const {
    DRAKE_BODY_THROW_IF_NOT_FINALIZED();
    DRAKE_DEMAND(is_floating());
    if (has_quaternion_dofs()) {
      DRAKE_DEMAND(0 <= position_index_in_body && position_index_in_body < 7);
    } else {
      DRAKE_DEMAND(0 <= position_index_in_body && position_index_in_body < 6);
    }
    return this->get_parent_tree().get_mobilizer(
      topology_.inboard_mobilizer).position_suffix(position_index_in_body);
  }

  /// Returns a string suffix (e.g. to be appended to the name()) to identify
  /// the `k`th velocity in the floating base. @p velocity_index_in_body must
  /// be in [0,6).
  /// @throws std::exception if called pre-finalize
  /// @pre `this` is a floating body
  /// @see is_floating(), MultibodyPlant::Finalize()
  std::string floating_velocity_suffix(int velocity_index_in_body) const {
    DRAKE_BODY_THROW_IF_NOT_FINALIZED();
    DRAKE_DEMAND(is_floating());
    DRAKE_DEMAND(0 <= velocity_index_in_body && velocity_index_in_body < 6);
    return this->get_parent_tree().get_mobilizer(
      topology_.inboard_mobilizer).velocity_suffix(velocity_index_in_body);
  }

  /// Returns the default mass (not Context dependent) for `this` body.
  /// In general, a body's mass can be a Context-dependent parameter that is
  /// returned by the method get_mass(). When a body's mass is a parameter, the
  /// value returned by default_mass() is used to initialize the mass parameter
  /// in the Context.
  virtual double default_mass() const = 0;

  /// Returns the default rotational inertia (not Context dependent) for `this`
  /// body B's about Bo (B's origin), expressed in B (this body's frame).
  /// @retval I_BBo_B body B's rotational inertia about Bo, expressed in B.
  virtual RotationalInertia<double> default_rotational_inertia() const = 0;

  /// (Advanced) Returns the mass of this body stored in `context`.
  virtual const T& get_mass(
      const systems::Context<T>& context) const = 0;

  /// (Advanced) Computes the center of mass `p_BoBcm_B` (or `p_Bcm` for short)
  /// of this body measured from this body's frame origin `Bo` and expressed in
  /// the body frame B.
  virtual const Vector3<T> CalcCenterOfMassInBodyFrame(
      const systems::Context<T>& context) const = 0;

  /// Calculates v_WBcm, Bcm's translational velocity in the world frame W.
  /// @param[in] context The context contains the state of the model.
  /// @retval v_WBcm_W Bcm's (`this` body's center of mass) translational
  /// velocity in the world frame W, expressed in W.
  virtual Vector3<T> CalcCenterOfMassTranslationalVelocityInWorld(
      const systems::Context<T>& context) const = 0;

  /// (Advanced) Computes the SpatialInertia `I_BBo_B` of `this` body about its
  /// frame origin `Bo` (not necessarily its center of mass) and expressed in
  /// its body frame `B`. The spatial inertia of a RigidBody in its body frame
  /// is constant, but may need to be calculated from Parameters.
  virtual SpatialInertia<T> CalcSpatialInertiaInBodyFrame(
      const systems::Context<T>& context) const = 0;

  /// Returns the pose `X_WB` of this body B in the world frame W as a function
  /// of the state of the model stored in `context`.
  const math::RigidTransform<T>& EvalPoseInWorld(
      const systems::Context<T>& context) const {
    return this->get_parent_tree().EvalBodyPoseInWorld(context, *this);
  }

  /// Evaluates V_WB, `this` body B's spatial velocity in the world frame W.
  /// @param[in] context Contains the state of the model.
  /// @retval V_WB_W `this` body B's spatial velocity in the world frame W,
  /// expressed in W (for point Bo, the body frame's origin).
  const SpatialVelocity<T>& EvalSpatialVelocityInWorld(
      const systems::Context<T>& context) const {
    return this->get_parent_tree().EvalBodySpatialVelocityInWorld(
        context, *this);
  }

  /// Evaluates A_WB, `this` body B's spatial acceleration in the world frame W.
  /// @param[in] context Contains the state of the model.
  /// @retval A_WB_W `this` body B's spatial acceleration in the world frame W,
  /// expressed in W (for point Bo, the body's origin).
  /// @note When cached values are out of sync with the state stored in context,
  /// this method performs an expensive forward dynamics computation, whereas
  /// once evaluated, successive calls to this method are inexpensive.
  const SpatialAcceleration<T>& EvalSpatialAccelerationInWorld(
      const systems::Context<T>& context) const {
    const MultibodyPlant<T>& parent_plant = this->GetParentPlant();
    return parent_plant.EvalBodySpatialAccelerationInWorld(context, *this);
  }

  /// Gets the sptatial force on `this` body B from `forces` as F_BBo_W:
  /// applied at body B's origin Bo and expressed in world frame W.
  const SpatialForce<T>& GetForceInWorld(
      const systems::Context<T>&, const MultibodyForces<T>& forces) const {
    DRAKE_THROW_UNLESS(
        forces.CheckHasRightSizeForModel(this->get_parent_tree()));
    return forces.body_forces()[mobod_index()];
  }

  /// Adds the spatial force on `this` body B, applied at body B's origin Bo and
  /// expressed in the world frame W into `forces`.
  void AddInForceInWorld(const systems::Context<T>&,
                         const SpatialForce<T>& F_Bo_W,
                         MultibodyForces<T>* forces) const {
    DRAKE_THROW_UNLESS(forces != nullptr);
    DRAKE_THROW_UNLESS(
        forces->CheckHasRightSizeForModel(this->get_parent_tree()));
    forces->mutable_body_forces()[mobod_index()] += F_Bo_W;
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
  /// Creates a %Body named `name` in model instance `model_instance`.
  /// The `name` must not be empty.
  Body(const std::string& name, ModelInstanceIndex model_instance)
      : MultibodyElement<T>(model_instance),
        name_(internal::DeprecateWhenEmptyName(name, "Body")),
        body_frame_(*this) {}

  /// Called by DoDeclareParameters(). Derived classes may choose to override
  /// to declare their sub-class specific parameters.
  virtual void DoDeclareBodyParameters(internal::MultibodyTreeSystem<T>*) {}

  /// Called by DoSetDefaultParameters(). Derived classes may choose to override
  /// to set their sub-class specific parameters.
  virtual void DoSetDefaultBodyParameters(systems::Parameters<T>*) const {}

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

  // Implementation for MultibodyElement::DoDeclareParameters().
  void DoDeclareParameters(
      internal::MultibodyTreeSystem<T>* tree_system) final {
    DoDeclareBodyParameters(tree_system);
  }

  // Implementation for MultibodyElement::DoSetDefaultParameters().
  void DoSetDefaultParameters(systems::Parameters<T>* parameters) const final {
    DoSetDefaultBodyParameters(parameters);
  }

  // MultibodyTree has access to the mutable BodyFrame through BodyAttorney.
  BodyFrame<T>& get_mutable_body_frame() {
    return body_frame_;
  }

  // A string identifying the body in its model.
  // Within a MultibodyPlant model instance this string is guaranteed to be
  // unique by MultibodyPlant's API.
  const std::string name_;

  // Body frame associated with this body.
  BodyFrame<T> body_frame_;

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
