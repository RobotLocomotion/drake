#pragma once

#include <memory>
#include <string>
#include <vector>

#include "drake/common/default_scalars.h"
#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/drake_deprecated.h"
#include "drake/common/unused.h"
#include "drake/multibody/tree/acceleration_kinematics_cache.h"
#include "drake/multibody/tree/body.h"
#include "drake/multibody/tree/multibody_tree_system.h"
#include "drake/multibody/tree/parameter_conversion.h"
#include "drake/multibody/tree/position_kinematics_cache.h"
#include "drake/multibody/tree/spatial_inertia.h"
#include "drake/multibody/tree/velocity_kinematics_cache.h"

namespace drake {
namespace multibody {

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
class RigidBody : public Body<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(RigidBody)

  /// Constructs a %RigidBody named `body_name` with the given default
  /// SpatialInertia.
  ///
  /// @param[in] body_name
  ///   A name associated with `this` body.
  /// @param[in] M_BBo_B
  ///   Spatial inertia of `this` body B about the frame's origin `Bo` and
  ///   expressed in the body frame B.
  /// @note See @ref multibody_spatial_inertia for details on the monogram
  /// notation used for spatial inertia quantities.
  RigidBody(const std::string& body_name,
            const SpatialInertia<double>& M_BBo_B);

  DRAKE_DEPRECATED("2022-12-01",
      "The body_name parameter to the RigidBody constructor is now required.")
  explicit RigidBody(const SpatialInertia<double>& M_BBo_B);

  /// Constructs a %RigidBody named `body_name` with the given default
  /// SpatialInertia.
  ///
  /// @param[in] body_name
  ///   A name associated with `this` body.
  /// @param[in] model_instance
  ///   The model instance associated with `this` body.
  /// @param[in] M_BBo_B
  ///   Spatial inertia of `this` body B about the frame's origin `Bo` and
  ///   expressed in the body frame B.
  /// @note See @ref multibody_spatial_inertia for details on the monogram
  /// notation used for spatial inertia quantities.
  RigidBody(const std::string& body_name,
            ModelInstanceIndex model_instance,
            const SpatialInertia<double>& M_BBo_B);

  /// There are no flexible degrees of freedom associated with a rigid body and
  /// therefore this method returns zero. By definition, a rigid body has no
  /// state associated with flexible deformations.
  int get_num_flexible_positions() const final { return 0; }

  /// There are no flexible degrees of freedom associated with a rigid body and
  /// therefore this method returns zero. By definition, a rigid body has no
  /// state associated with flexible deformations.
  int get_num_flexible_velocities() const final { return 0; }

  /// Returns this rigid body's default mass, which is initially supplied at
  /// construction when specifying this rigid body's SpatialInertia.
  /// @note In general, a rigid body's mass can be a constant property stored in
  /// this rigid body's %SpatialInertia or a parameter that is stored in a
  /// Context. The default constant mass value is used to initialize the mass
  /// parameter in the Context.
  double default_mass() const final {
    return default_spatial_inertia_.get_mass();
  }

  /// Returns the default value of this rigid body's center of mass as measured
  /// and expressed in this body's frame. This value is initially supplied at
  /// construction when specifying this body's SpatialInertia.
  /// @retval p_BoBcm_B The position of this rigid body B's center of mass `Bcm`
  /// measured from Bo (B's frame origin) and expressed in B (body B's frame).
  const Vector3<double>& default_com() const {
    return default_spatial_inertia_.get_com();
  }

  /// Returns the default value of this body B's unit inertia about Bo (body B's
  /// origin), expressed in B (this body's frame). This value is initially
  /// supplied at construction when specifying this body's SpatialInertia.
  /// @retval G_BBo_B rigid body B's unit inertia about Bo, expressed in B.
  const UnitInertia<double>& default_unit_inertia() const {
    return default_spatial_inertia_.get_unit_inertia();
  }

  /// Gets the default value of this body B's rotational inertia about Bo
  /// (B's origin), expressed in B (this body's frame). This value is calculated
  /// from the SpatialInertia supplied at construction of this body.
  /// @retval I_BBo_B body B's rotational inertia about Bo, expressed in B.
  RotationalInertia<double> default_rotational_inertia() const final {
    return default_spatial_inertia_.CalcRotationalInertia();
  }

  /// Gets the default value of this body B's spatial inertia about Bo
  /// (B's origin) and expressed in B (this body's frame).
  /// @retval M_BBo_B body B's spatial inertia about Bo, expressed in B.
  const SpatialInertia<double>& default_spatial_inertia() const {
    return default_spatial_inertia_;
  }

  /// Gets this body's mass from the given context.
  /// @param[in] context contains the state of the multibody system.
  /// @pre the context makes sense for use by `this` RigidBody.
  const T& get_mass(const systems::Context<T>& context) const final {
    const systems::BasicVector<T>& spatial_inertia_parameter =
        context.get_numeric_parameter(spatial_inertia_parameter_index_);
    return internal::parameter_conversion::GetMass(spatial_inertia_parameter);
  }

  /// Gets this body's center of mass position from the given context.
  /// @param[in] context contains the state of the multibody system.
  /// @returns p_BoBcm_B position vector from Bo (this rigid body B's origin)
  /// to Bcm (B's center of mass), expressed in B.
  /// @pre the context makes sense for use by `this` RigidBody.
  const Vector3<T> CalcCenterOfMassInBodyFrame(
      const systems::Context<T>& context) const final {
    const systems::BasicVector<T>& spatial_inertia_parameter =
        context.get_numeric_parameter(spatial_inertia_parameter_index_);
    return internal::parameter_conversion::GetCenterOfMass(
        spatial_inertia_parameter);
  }

  /// Calculates Bcm's translational velocity in the world frame W.
  /// @param[in] context The context contains the state of the model.
  /// @retval v_WBcm_W The translational velocity of Bcm (`this` rigid body's
  /// center of mass) in the world frame W, expressed in W.
  Vector3<T> CalcCenterOfMassTranslationalVelocityInWorld(
      const systems::Context<T>& context) const override {
    const Body<T>& body_B = *this;
    const Frame<T>& frame_B = body_B.body_frame();

    // Form frame_B's spatial velocity in the world frame W, expressed in W.
    const SpatialVelocity<T>& V_WBo_W =
        body_B.EvalSpatialVelocityInWorld(context);

    // Form v_WBcm_W, Bcm's translational velocity in frame W, expressed in W.
    const Vector3<T> p_BoBcm_B = CalcCenterOfMassInBodyFrame(context);
    const math::RotationMatrix<T> R_WB =
        frame_B.CalcRotationMatrixInWorld(context);
    const Vector3<T> p_BoBcm_W = R_WB * p_BoBcm_B;
    const Vector3<T> v_WBcm_W = V_WBo_W.Shift(p_BoBcm_W).translational();
    return v_WBcm_W;
  }

  // TODO(joemasterjohn): Speed this up when we can store a reference to a
  // SpatialInertia<T> as an abstract parameter.

  /// Gets this body's spatial inertia about its origin from the given context.
  /// @param[in] context contains the state of the multibody system.
  /// @returns M_BBo_B spatial inertia of this rigid body B about Bo (B's
  /// origin), expressed in B. M_BBo_B contains properties related to B's mass,
  /// the position vector from Bo to Bcm (B's center of mass), and G_BBo_B
  /// (B's unit inertia about Bo expressed in B).
  /// @pre the context makes sense for use by `this` RigidBody.
  SpatialInertia<T> CalcSpatialInertiaInBodyFrame(
      const systems::Context<T>& context) const override {
    const systems::BasicVector<T>& spatial_inertia_parameter =
        context.get_numeric_parameter(spatial_inertia_parameter_index_);
    return internal::parameter_conversion::ToSpatialInertia(
        spatial_inertia_parameter);
  }

  /// For `this` rigid body B, sets its mass stored in @p context to @p mass.
  /// @param[out] context contains the state of the multibody system.
  /// @param[in] mass mass of `this` rigid body B.
  /// @note This function changes `this` body B's mass and appropriately scales
  /// I_BBo_B (B's rotational inertia about Bo, expressed in B).
  /// @pre the context makes sense for use by `this` RigidBody.
  /// @throws std::exception if context is null.
  void SetMass(systems::Context<T>* context, const T& mass) const {
    DRAKE_THROW_UNLESS(context != nullptr);
    systems::BasicVector<T>& spatial_inertia_parameter =
        context->get_mutable_numeric_parameter(
            spatial_inertia_parameter_index_);
    spatial_inertia_parameter.SetAtIndex(
        internal::parameter_conversion::SpatialInertiaIndex::k_mass, mass);
  }

  /// For `this` rigid body B, sets its center of mass position stored in
  /// @p context to @p com.
  /// @param[out] context contains the state of the multibody system.
  /// @param[in] com position vector from Bo (B's origin) to Bcm
  /// (B's center of mass), expressed in B.
  /// @note This function changes `this` body B's center of mass position
  /// without modifying I_BBo_B (B's rotational inertia about Bo, expressed
  /// in B). If needed, use SetSpatialInertiaInBodyFrame().
  /// @pre the context makes sense for use by `this` RigidBody.
  /// @throws std::exception if context is null.
  void SetCenterOfMassInBodyFrame(systems::Context<T>* context,
                                  const Vector3<T>& com) const {
    DRAKE_THROW_UNLESS(context != nullptr);
    const T& x = com(0);
    const T& y = com(1);
    const T& z = com(2);
    systems::BasicVector<T>& spatial_inertia_parameter =
        context->get_mutable_numeric_parameter(
            spatial_inertia_parameter_index_);
    spatial_inertia_parameter.SetAtIndex(
        internal::parameter_conversion::SpatialInertiaIndex::k_com_x, x);
    spatial_inertia_parameter.SetAtIndex(
        internal::parameter_conversion::SpatialInertiaIndex::k_com_y, y);
    spatial_inertia_parameter.SetAtIndex(
        internal::parameter_conversion::SpatialInertiaIndex::k_com_z, z);
  }

  /// For `this` rigid body B, sets its spatial inertia that is stored in
  /// @p context to @p M_Bo_B.
  /// @param[out] context contains the state of the multibody system.
  /// @param[in] M_Bo_B spatial inertia of this rigid body B about Bo (B's
  /// origin), expressed in B. M_Bo_B contains properties related to B's mass,
  /// the position vector from Bo to Bcm (B's center of mass), and G_Bo_B
  /// (B's unit inertia about Bo expressed in B).
  /// @pre the context makes sense for use by `this` RigidBody.
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
  // TODO(amcastro-tri) When cache entries are in the context, replace this
  // method by Body<T>::get_pose_in_world(const Context<T>&).
  //----------------------------------------------------------------------------
  const math::RigidTransform<T>& get_pose_in_world(
      const internal::PositionKinematicsCache<T>& pc) const {
    return pc.get_X_WB(this->node_index());
  }

  /// (Advanced) Extract the rotation matrix relating the world frame to this
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

  /// (Advanced) Returns V_WB, `this` rigid body B's spatial velocity in
  /// the world frame W.
  /// @param[in] vc velocity kinematics cache.
  /// @retval V_WB_W `this` rigid body B's spatial velocity in the world
  /// frame W, expressed in W (for point Bo, the body frame's origin).
  // TODO(amcastro-tri) When cache entries are in the context, replace this
  //  method by Body<T>::get_spatial_velocity_in_world(const Context<T>&).
  //----------------------------------------------------------------------------
  const SpatialVelocity<T>& get_spatial_velocity_in_world(
      const internal::VelocityKinematicsCache<T>& vc) const {
    return vc.get_V_WB(this->node_index());
  }

  /// (Advanced) Extract this body angular velocity in world, expressed in
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

  /// (Advanced) Returns A_WB, `this` rigid body B's spatial acceleration in
  /// the world frame W.
  /// @param[in] ac acceleration kinematics cache.
  /// @retval A_WB_W `this` rigid body B's spatial acceleration in the world
  /// frame W, expressed in W (for point Bo, the body frame's origin).
  // TODO(amcastro-tri) When cache entries are in the context, replace this
  // method by Body<T>::get_spatial_acceleration_in_world(const Context<T>&).
  //----------------------------------------------------------------------------
  const SpatialAcceleration<T>& get_spatial_acceleration_in_world(
      const internal::AccelerationKinematicsCache<T>& ac) const {
    return ac.get_A_WB(this->node_index());
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
  /// @param[in] vc velocity kinematics cache.
  /// @retval a_WBo_W acceleration of body origin Bo in world W, expressed in W.
  const Vector3<T>& get_origin_acceleration_in_world(
      const internal::AccelerationKinematicsCache<T>& ac) const {
    return get_spatial_acceleration_in_world(ac).translational();
  }
  ///@}

 protected:
  std::unique_ptr<Body<double>> DoCloneToScalar(
      const internal::MultibodyTree<double>& tree_clone) const final {
    return TemplatedDoCloneToScalar(tree_clone);
  }

  std::unique_ptr<Body<AutoDiffXd>> DoCloneToScalar(
      const internal::MultibodyTree<AutoDiffXd>& tree_clone) const final {
    return TemplatedDoCloneToScalar(tree_clone);
  }

  std::unique_ptr<Body<symbolic::Expression>> DoCloneToScalar(
      const internal::MultibodyTree<symbolic::Expression>& tree_clone) const
      final {
    return TemplatedDoCloneToScalar(tree_clone);
  }

  // Implementation for MultibodyElement::DoDeclareParameters().
  // RigidBody declares a single parameter for its SpatialInertia.
  void DoDeclareParameters(
      internal::MultibodyTreeSystem<T>* tree_system) override {
    // Declare parent classes' parameters
    Body<T>::DoDeclareParameters(tree_system);

    spatial_inertia_parameter_index_ = this->DeclareNumericParameter(
        tree_system, internal::parameter_conversion::ToBasicVector<T>(
                         default_spatial_inertia_.template cast<T>()));
  }

 private:
  // Helper method to make a clone templated on ToScalar.
  template <typename ToScalar>
  std::unique_ptr<Body<ToScalar>> TemplatedDoCloneToScalar(
      const internal::MultibodyTree<ToScalar>& tree_clone) const {
    unused(tree_clone);
    return std::make_unique<RigidBody<ToScalar>>(
        this->name(), default_spatial_inertia_);
  }

  // Spatial inertia about the body frame origin Bo, expressed in B.
  SpatialInertia<double> default_spatial_inertia_;

  // System parameter index for `this` bodies SpatialInertia stored in a
  // context.
  systems::NumericParameterIndex spatial_inertia_parameter_index_;
};

}  // namespace multibody
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::multibody::RigidBody)
