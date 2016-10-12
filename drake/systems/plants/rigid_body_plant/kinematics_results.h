#pragma once

#include <Eigen/Geometry>

#include "drake/common/drake_export.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/plants/RigidBodyTree.h"

namespace drake {
namespace systems {

// Forward declaration for KinematicsResults.
template <typename T> class RigidBodyPlant;

/// A class containing the kinematics results from a RigidBodyPlant system.
/// @tparam T The scalar type. Must be a valid Eigen scalar.
template <typename T>
class DRAKE_EXPORT KinematicsResults {
 public:
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

 private:
  // RigidBodyPlant is the only class allowed to update KinematicsResults
  // through UpdateFromContext().
  // TODO(amcastro-tri): when KinematicsResults can reference entries in the
  // cache this friendship and the method UpdateFromContext() won't be needed.
  friend class RigidBodyPlant<T>;

  // Only RigidBodyPlant can construct a KinematicsResults from the underlying
  // RigidBodyTree.
  // An alias to @tree is maintained so that the tree's lifetime must exceed
  // this object's lifetime.
  explicit KinematicsResults(const RigidBodyTree* tree);

  // Updates KinematicsResults from a context provided by RigidBodyPlant.
  // Only RigidBodyPlant has access to this method since it is a friend.
  // TODO(amcastro-tri): when KinematicsResults can reference entries in the
  // cache this method won't be needed.
  void UpdateFromContext(const Context<T>& context);

  const RigidBodyTree* tree_;
  KinematicsCache<T> kinematics_cache_;
};

}  // namespace systems
}  // namespace drake
