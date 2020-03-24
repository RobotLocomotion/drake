#pragma once

#include <memory>
#include <string>
#include <vector>

#include "drake/common/default_scalars.h"
#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/unused.h"
#include "drake/multibody/tree/acceleration_kinematics_cache.h"
#include "drake/multibody/tree/body.h"
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

  /// Constructs a %RigidBody with the given default SpatialInertia.
  /// @param[in] M_BBo_B
  ///   Spatial inertia of `this` body B about the frame's origin `Bo` and
  ///   expressed in the body frame B.
  /// @note See @ref multibody_spatial_inertia for details on the monogram
  /// notation used for spatial inertia quantities.
  explicit RigidBody(const SpatialInertia<double>& M_BBo_B);

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

  /// Returns the default value of this body's mass.  This value is initially
  /// supplied at construction when specifying this body's SpatialInertia.
  /// @returns This body's default mass.
  double default_mass() const {
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
  RotationalInertia<double> default_rotational_inertia()
      const {
    return default_spatial_inertia_.CalcRotationalInertia();
  }

  /// Gets the default value of this body B's spatial inertia about Bo
  /// (B's origin) and expressed in B (this body's frame).
  /// @retval M_BBo_B body B's spatial inertia about Bo, expressed in B.
  const SpatialInertia<double>& default_spatial_inertia() const {
    return default_spatial_inertia_;
  }

  T get_mass(const systems::Context<T>&) const final {
    return default_spatial_inertia_.get_mass();
  }

  const Vector3<T> CalcCenterOfMassInBodyFrame(
      const systems::Context<T>&) const final {
    return default_com().template cast<T>();
  }

  SpatialInertia<T> CalcSpatialInertiaInBodyFrame(
      const systems::Context<T>&) const override {
    return default_spatial_inertia_.cast<T>();
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

  /// (Advanced) Extract this body spatial velocity in world, expressed in
  /// world.
  /// @param[in] vc velocity kinematics cache.
  /// @retval V_WB_W rigid body B's spatial velocity in world W, expressed in W.
  // TODO(amcastro-tri) When cache entries are in the context, replace this
  // method by Body<T>::get_spatial_velocity_in_world(const Context<T>&).
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

  /// (Advanced) Extract this body spatial acceleration in world, expressed in
  /// world.
  /// @param[in] ac acceleration kinematics cache.
  /// @retval A_WB_W body B's spatial acceleration in world W, expressed in W.
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
};

}  // namespace multibody
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::multibody::RigidBody)
