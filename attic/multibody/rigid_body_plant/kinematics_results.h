#pragma once

#include <Eigen/Geometry>

#include "drake/common/drake_copyable.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/systems/framework/context.h"

namespace drake {
namespace systems {

// Forward declaration for KinematicsResults.
template <typename T>
class RigidBodyPlant;

/// A class containing the kinematics results from a RigidBodyPlant system.
/// @tparam T The scalar type. Must be a valid Eigen scalar.
///
/// Instantiated templates for the following ScalarTypes are provided:
///
///  - double
///  - AutoDiffXd
///
/// @note The templated ScalarTypes are used in the KinematicsCache, but all
/// KinematicsResults use RigidBodyTree<double>.  This effectively implies
/// that we can e.g. AutoDiffXd with respect to the configurations, but not
/// the RigidBodyTree parameters.
template <typename T>
class KinematicsResults {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(KinematicsResults)

  /// Constructs a KinematicsResults object associated with @param tree.
  /// An alias to @param tree is maintained so that the tree's lifetime must
  /// exceed this object's lifetime.
  explicit KinematicsResults(const RigidBodyTree<double>* tree);

  /// Updates the KinematicsResults object given the configuration vector
  /// @param q and velocity vector @param v.
  void Update(const Eigen::Ref<const VectorX<T>>& q,
              const Eigen::Ref<const VectorX<T>>& v);

  /// Returns the number of bodies in the kinematics results.
  int get_num_bodies() const;

  /// Returns the number of generalized positions.
  int get_num_positions() const;

  /// Returns the number of generalized velocities.
  int get_num_velocities() const;

  /// Returns the quaternion representation of the three dimensional orientation
  /// of body @p body_index in the world's frame.
  Quaternion<T> get_body_orientation(int body_index) const;

  /// Returns the three dimensional position of body @p body_index in world's
  /// frame.
  Vector3<T> get_body_position(int body_index) const;

  /// Returns the pose of body @p body with respect to the world.
  Isometry3<T> get_pose_in_world(const RigidBody<T>& body) const;

  /// Returns the twist of @p body with respect to the world, expressed in world
  /// frame.
  TwistVector<T> get_twist_in_world_frame(const RigidBody<T>& body) const;

  /// Returns the twist of @p body with respect to the world, expressed in world
  /// frame.
  TwistVector<T> get_twist_in_world_aligned_body_frame(
      const RigidBody<T>& body) const;

  // TODO(tkoolen) should pass in joint instead of body, but that's currently
  // not convenient.
  /// Returns the joint position vector associated with the joint between
  /// @p body and @p body's parent.
  Eigen::VectorBlock<const VectorX<T>> get_joint_position(
      const RigidBody<T>& body) const;

  // TODO(tkoolen): should pass in joint instead of body, but that's currently
  // not convenient.
  /// Returns the joint velocity vector associated with the joint between
  /// @p body and @p body's parent.
  Eigen::VectorBlock<const VectorX<T>> get_joint_velocity(
      const RigidBody<T>& body) const;

  const KinematicsCache<T>& get_cache() const {
    return kinematics_cache_;
  }

 private:
  // RigidBodyPlant is the only class allowed to update KinematicsResults
  // through UpdateFromContext().
  // TODO(amcastro-tri): when KinematicsResults can reference entries in the
  // cache this friendship and the method UpdateFromContext() won't be needed.
  friend class RigidBodyPlant<T>;

  // Updates KinematicsResults from a context provided by RigidBodyPlant.
  // Only RigidBodyPlant has access to this method since it is a friend.
  // TODO(amcastro-tri): when KinematicsResults can reference entries in the
  // cache this method won't be needed.
  void UpdateFromContext(const Context<T>& context);

  // Note that the tree is a double, but the KinematicCache will be T.
  const RigidBodyTree<double>* tree_;
  KinematicsCache<T> kinematics_cache_;
};

}  // namespace systems
}  // namespace drake
