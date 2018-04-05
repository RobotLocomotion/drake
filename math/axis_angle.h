/// @file
/// Utilities for arithmetic on axis-angle rotations.
// TODO(mitiguy) Replace the functionality in this file with related methods
// in Eigen::AngleAxis and Eigen::Quaternion and then delete this file.

#pragma once

#include <cmath>

#include <Eigen/Dense>

#include "drake/common/eigen_types.h"
#include "drake/math/quaternion.h"

namespace drake {
namespace math {
/**
 * Converts Drake's axis-angle representation to quaternion representation.
 * @param axis_angle. A 4 x 1 column vector [axis; angle]. axis is the unit
 * length rotation axis, angle is within [-PI, PI].
 * @return A 4 x 1 column vector, the unit length quaternion [w; x; y; z].
 */
template <typename T>
Vector4<T> axis2quat(const Eigen::AngleAxis<T>& angle_axis) {
  const Eigen::Quaternion<T> q(angle_axis);
  return Vector4<T>(q.w(), q.x(), q.y(), q.z());
}

/**
 * Converts Drake's axis-angle representation to body fixed z-y'-x'' Euler
 * angles,
 * or equivalently space fixed x-y-z Euler angles.
 * @param axis_angle. A 4 x 1 column vector [axis; angle]. axis is the unit
 * length rotation axis, angle is within [-PI, PI]
 * @return A 3 x 1 vector [roll, pitch, yaw]. Represents the body-fixed z-y'-x''
 * rotation with (yaw, pitch, roll) angles respectively. @see rpy2rotmat
 */
template <typename T>
Vector3<T> axis2rpy(const Eigen::AngleAxis<T>& angle_axis) {
  const Eigen::Quaternion<T> q(angle_axis);
  const Vector4<T> q_drake(q.w(), q.x(), q.y(), q.z());
  return quat2rpy(q_drake);
}

}  // namespace math
}  // namespace drake
