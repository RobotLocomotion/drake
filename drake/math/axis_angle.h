/// @file
/// Utilities for arithmetic on axis-angle rotations.

#pragma once

#include <cmath>

#include <Eigen/Dense>

#include "drake/common/eigen_types.h"
#include "drake/math/rotation_matrix.h"

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
 * Converts Drake's axis-angle representation to rotation matrix.
 * @param axis_angle. A 4 x 1 column vector [axis; angle]. axis is the unit
 * length rotation axis, angle is within [-PI, PI].
 * @return A 3 x 3 rotation matrix.
 */
template <typename Derived>
Matrix3<typename Derived::Scalar> axis2rotmat(
    const Eigen::MatrixBase<Derived>& axis_angle) {
  // TODO(hongkai.dai@tri.global): Switch to Eigen's AngleAxis when we fix
  // the range problem in Eigen
  using std::cos;
  using std::sin;
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Eigen::MatrixBase<Derived>, 4);
  const auto& axis =
      (axis_angle.template head<3>()) / (axis_angle.template head<3>()).norm();
  const auto& theta = axis_angle(3);
  auto x = axis(0);
  auto y = axis(1);
  auto z = axis(2);
  auto ctheta = cos(theta);
  auto stheta = sin(theta);
  auto c = 1 - ctheta;
  Matrix3<typename Derived::Scalar> R;
  R << ctheta + x * x * c, x * y * c - z * stheta, x * z * c + y * stheta,
      y * x * c + z * stheta, ctheta + y * y * c, y * z * c - x * stheta,
      z * x * c - y * stheta, z * y * c + x * stheta, ctheta + z * z * c;

  return R;
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
  return rotmat2rpy(axis2rotmat(axis_angle));
}

}  // namespace math
}  // namespace drake
