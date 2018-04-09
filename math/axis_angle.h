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
 * Converts Drake's axis-angle representation to Eigen's AngleAxis object.
 * @param axis_angle A 4 x 1 column vector [axis; angle]. axis is the unit
 * length rotation axis. angle is within [-PI, PI].
 * @return An Eigen::AngleAxis object.
 */
template <typename Derived>
Eigen::AngleAxis<typename Derived::Scalar> axisToEigenAngleAxis(
    const Eigen::MatrixBase<Derived>& axis_angle) {
  // TODO(hongkai.dai@tri.global): Switch to Eigen's AngleAxis when we fix
  // the range problem in Eigen
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Eigen::MatrixBase<Derived>, 4);
  return Eigen::AngleAxis<typename Derived::Scalar>(
      axis_angle(3), axis_angle.template head<3>());
}

/**
 * Converts Drake's axis-angle representation to quaternion representation.
 * @param axis_angle. A 4 x 1 column vector [axis; angle]. axis is the unit
 * length rotation axis, angle is within [-PI, PI].
 * @return A 4 x 1 column vector, the unit length quaternion [w; x; y; z].
 */
template <typename Derived>
Vector4<typename Derived::Scalar> axis2quat(
    const Eigen::MatrixBase<Derived>& axis_angle) {
  // TODO(hongkai.dai@tri.global): Switch to Eigen's AngleAxis when we fix
  // the range problem in Eigen
  using std::cos;
  using std::sin;
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Eigen::MatrixBase<Derived>, 4);
  auto axis = axis_angle.template head<3>();
  auto angle = axis_angle(3);
  auto arg = 0.5 * angle;
  auto c = cos(arg);
  auto s = sin(arg);
  Vector4<typename Derived::Scalar> ret;
  ret << c, s * axis;
  return ret;
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
template <typename Derived>
Vector3<typename Derived::Scalar> axis2rpy(
    const Eigen::MatrixBase<Derived>& axis_angle) {
  // TODO(hongkai.dai@tri.global): Switch to Eigen's AngleAxis when we fix
  // the range problem in Eigen
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Eigen::MatrixBase<Derived>, 4);
  return quat2rpy(axis2quat(axis_angle));
}

}  // namespace math
}  // namespace drake
