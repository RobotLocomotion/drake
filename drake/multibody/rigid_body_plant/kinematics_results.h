#pragma once

#include <Eigen/Geometry>

#include "drake/multibody/rigid_body_tree.h"
#include "drake/systems/framework/context.h"

namespace drake {
namespace systems {

// Forward declaration for KinematicsResults.
template <typename T>
class RigidBodyPlant;

/// A class containing the kinematics results from a RigidBodyPlant system.
/// @tparam T The scalar type. Must be a valid Eigen scalar.
template <typename T>
class KinematicsResults {
 public:
  /// Constructs a KinematicsResults object associated with @param tree.
  /// An alias to @param tree is maintained so that the tree's lifetime must
  /// exceed this object's lifetime.
  explicit KinematicsResults(const RigidBodyTree<T>* tree);

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

  /// Returns the joint position vector associated with the joint between
  /// @p body and @p body's parent.
  /// TODO(tkoolen) should pass in joint instead of body, but that's currently
  /// not convenient.
  Eigen::VectorBlock<const VectorX<T>> get_joint_position(
      const RigidBody<T>& body) const;

  /// Returns the joint velocity vector associated with the joint between
  /// @p body and @p body's parent.
  /// TODO(tkoolen) should pass in joint instead of body, but that's currently
  /// not convenient.
  Eigen::VectorBlock<const VectorX<T>> get_joint_velocity(
      const RigidBody<T>& body) const;

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

  const RigidBodyTree<T>* tree_;
  KinematicsCache<T> kinematics_cache_;
};

}  // namespace systems
}  // namespace drake
