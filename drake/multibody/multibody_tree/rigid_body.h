#pragma once

#include <memory>
#include <vector>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/unused.h"
#include "drake/multibody/multibody_tree/body.h"
#include "drake/multibody/multibody_tree/multibody_tree.h"
#include "drake/multibody/multibody_tree/multibody_tree_element.h"
#include "drake/multibody/multibody_tree/spatial_inertia.h"

namespace drake {
namespace multibody {

// Forward declarations.
template<typename T> class MultibodyTree;

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
/// @tparam T The scalar type. Must be a valid Eigen scalar.
///
/// Instantiated templates for the following kinds of T's are provided:
/// - double
/// - AutoDiffXd
///
/// They are already available to link against in the containing library.
/// No other values for T are currently supported.
template <typename T>
class RigidBody : public Body<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(RigidBody)

  /// Constructs a %RigidBody with the given default SpatialInertia.
  /// @param[in] M_BBo_B Spatial inertia of `this` body B about the frame's
  ///            origin `Bo`, expressed in the body frame B.
  /// @note See @ref multibody_spatial_inertia for details on the monogram
  /// notation used for spatial inertia quantities.
  explicit RigidBody(const SpatialInertia<double> M_BBo_B);

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
  double get_default_mass() const {
    return default_spatial_inertia_.get_mass();
  }

  /// Returns the default value of this rigid body's center of mass as measured
  /// and expressed in this body's frame. This value is initially supplied at
  /// construction when specifying this body's SpatialInertia.
  /// @retval p_BoBcm_B The position of this rigid body B's center of mass `Bcm`
  /// measured from Bo (B's frame origin) and expressed in B (body B's frame).
  const Vector3<double>& get_default_com() const {
    return default_spatial_inertia_.get_com();
  }

  /// Returns the default value of this body B's unit inertia about Bo (body B's
  /// origin), expressed in B (this body's frame). This value is initially
  /// supplied at construction when specifying this body's SpatialInertia.
  /// @retval G_BBo_B rigid body B's unit inertia about Bo, expressed in B.
  const UnitInertia<double>& get_default_unit_inertia() const {
    return default_spatial_inertia_.get_unit_inertia();
  }

  /// Calculates the default value of this body B's rotational inertia about Bo
  /// (B's origin), expressed in B (this body's frame). This value is calculated
  /// from the SpatialInertia supplied at construction of this body.
  /// @retval I_BBo_B body B's rotational inertia about Bo, expressed in B.
  RotationalInertia<double> CalcDefaultRotationalInertiaAboutBo() const {
    return default_spatial_inertia_.CalcRotationalInertia();
  }

  SpatialInertia<T> CalcSpatialInertiaInBodyFrame(
      const MultibodyTreeContext<T>&) const override {
    return default_spatial_inertia_.cast<T>();
  }

  /// Extract this body's pose in world (from the position kinematics).
  /// @param[in] pc position kinematics cache.
  /// @retval X_WB pose relating W (world frame) to B (this body's frame).
  // TODO(amcastro-tri) When cache entries are in the context, replace this
  // method by Body<T>::get_pose_in_world(const Context<T>&).
  //----------------------------------------------------------------------------
  const Isometry3<T>& get_pose_in_world(
      const PositionKinematicsCache<T>& pc) const {
    const BodyNodeIndex body_node_index = get_body_node_index();
    return pc.get_X_WB(body_node_index);
  }

  /// Extract the rotation matrix relating the world frame to this body's frame.
  /// @param[in] pc position kinematics cache.
  /// @retval R_WB rotation matrix relating unit vectors Wx, Wy, Wz fixed in
  ///    world frame W to unit vectors Bx, By, Bz fixed in this rigid body B.
  const Matrix3<T> get_world_to_body_rotation_matrix(
      const PositionKinematicsCache<T>& pc) const {
    const Isometry3<T>& X_WB = get_pose_in_world(pc);
    return X_WB.linear();
  }

  /// Extract the position vector from World origin to this body's origin,
  /// expressed in world.
  /// @param[in] pc position kinematics cache.
  /// @retval p_WoBo_W position vector from Wo (world origin) to
  ///         Bo (this body's origin) expressed in W (world).
  const Vector3<T> get_position_from_world_origin_expressed_in_world(
      const PositionKinematicsCache<T>& pc) const {
    const Isometry3<T>& X_WB = get_pose_in_world(pc);
    return X_WB.translation();
  }

  /// Extract this body spatial velocity in world, expressed in world.
  /// @param[in] vc velocity kinematics cache.
  /// @retval V_WB_W rigid body B's spatial velocity in world W, expressed in W.
  // TODO(amcastro-tri) When cache entries are in the context, replace this
  // method by Body<T>::get_spatial_velocity_in_world(const Context<T>&).
  //----------------------------------------------------------------------------
  const SpatialVelocity<T>&
  get_spatial_velocity_in_world_expressed_in_world(
      const VelocityKinematicsCache<T>& vc) const {
    const BodyNodeIndex body_node_index = get_body_node_index();
    return vc.get_V_WB(body_node_index);
  }

  /// Extract this body angular velocity in world, expressed in world.
  /// @param[in] vc velocity kinematics cache.
  /// @retval w_WB_W rigid body B's angular velocity in world W, expressed in W.
  const Vector3<T>& get_angular_velocity_in_world_expressed_in_world(
      const VelocityKinematicsCache<T>& vc) const {
    const SpatialVelocity<T>& V_WB_W =
        get_spatial_velocity_in_world_expressed_in_world(vc);
    return V_WB_W.rotational();
  }

  /// Extract the velocity of this body's origin in world, expressed in world.
  /// @param[in] vc velocity kinematics cache.
  /// @retval v_WBo_W velocity of Bo (body origin) in world W, expressed in W.
  const Vector3<T>& get_origin_velocity_in_world_expressed_in_world(
      const VelocityKinematicsCache<T>& vc) const {
    const SpatialVelocity<T>& V_WB_W =
        get_spatial_velocity_in_world_expressed_in_world(vc);
    return V_WB_W.translational();
  }

  /// Extract this body spatial acceleration in world, expressed in world.
  /// @param[in] ac acceleration kinematics cache.
  /// @retval A_WB_W body B's spatial acceleration in world W, expressed in W.
  // TODO(amcastro-tri) When cache entries are in the context, replace this
  // method by Body<T>::get_spatial_acceleration_in_world(const Context<T>&).
  //----------------------------------------------------------------------------
  const SpatialAcceleration<T>&
  get_spatial_acceleration_in_world_expressed_in_world(
      const AccelerationKinematicsCache<T>& ac) const {
    const BodyNodeIndex body_node_index = get_body_node_index();
    return ac.get_A_WB(body_node_index);
  }

  /// Extract this body's angular acceleration in world, expressed in world.
  /// @param[in] ac velocity kinematics cache.
  /// @retval alpha_WB_W B's angular acceleration in world W, expressed in W.
  const Vector3<T>& get_angular_acceleration_in_world_expressed_in_world(
      const AccelerationKinematicsCache<T>& ac) const {
    const SpatialAcceleration<T>& A_WB_W =
        get_spatial_acceleration_in_world_expressed_in_world(ac);
    return A_WB_W.rotational();
  }

  /// Extract acceleration of this body's origin in world, expressed in world.
  /// @param[in] vc velocity kinematics cache.
  /// @retval a_WBo_W acceleration of Bo (body origin) in world W, expressed in W.
  const Vector3<T>& get_origin_acceleration_in_world_expressed_in_world(
      const AccelerationKinematicsCache<T>& ac) const {
    const SpatialAcceleration<T>& A_WB_W =
        get_spatial_acceleration_in_world_expressed_in_world(ac);
    return A_WB_W.translational();
  }

 protected:
  std::unique_ptr<Body<double>> DoCloneToScalar(
      const MultibodyTree<double>& tree_clone) const final {
    return TemplatedDoCloneToScalar(tree_clone);
  }

  std::unique_ptr<Body<AutoDiffXd>> DoCloneToScalar(
      const MultibodyTree<AutoDiffXd>& tree_clone) const final {
    return TemplatedDoCloneToScalar(tree_clone);
  }

 private:
  // Helper method to make a clone templated on ToScalar.
  template <typename ToScalar>
  std::unique_ptr<Body<ToScalar>> TemplatedDoCloneToScalar(
      const MultibodyTree<ToScalar>& tree_clone) const {
    unused(tree_clone);
    return std::make_unique<RigidBody<ToScalar>>(default_spatial_inertia_);
  }

  // This helper method returns a body's BodyNodeIndex.
  BodyNodeIndex get_body_node_index() const { return this->get_node_index(); }

  // Spatial inertia about the body frame origin Bo, expressed in B.
  SpatialInertia<double> default_spatial_inertia_;
};

}  // namespace multibody
}  // namespace drake
