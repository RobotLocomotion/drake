/**
 * @file
 * This file includes conversions from roll-pitch-yaw representation,
 * that *do* depend on quaternion.h.
 *
 * The old file roll_pitch_yaw.h was separated into two half-files, namely:
 * roll_pitch_yaw_not_using_quaternion.h and roll_pitch_yaw_using_quaternion.h,
 * so quaternion.h can include roll_pitch_yaw_not_using_quaternion.h
 * to call rpy2rotmat in quaternion.h, without circular dependency problems.
 *
 * Most users should just include roll_pitch_yaw.h (ignore the half-files).
 */
#pragma once

#include "drake/math/quaternion.h"
#include "drake/math/roll_pitch_yaw_not_using_quaternion.h"

namespace drake {
namespace math {
/**
 * Computes angle-axis representation from Euler angles.
 * @param rpy A 3 x 1 vector. The Euler angles about Body-fixed z-y'-x'' axes
 * by angles [rpy(2), rpy(1), rpy(0)].
 * @return A 4 x 1 angle-axis representation `a`, with `a.head<3>()` being the
 * rotation axis,  `a(3)` being the rotation angle
 * @see rpy2rotmat
 */
template <typename Derived>
Vector4<typename Derived::Scalar> rpy2axis(
    const Eigen::MatrixBase<Derived>& rpy) {
  // TODO(hongkai.dai@tri.global): Switch to Eigen's EulerAngles when we fix
  // the range problem in Eigen
  return quat2axis(rpy2quat(rpy));
}

/**
 * Computes the Quaternion representation of a rotation given the set of Euler
 * angles describing this rotation. These angles follow the Tait–Bryan formalism
 * about body-fixed z-y'-x'' axes. This convention is equivalent to a
 * space-fixed x-y-z sequence.
 * @param rpy A vector conveniently packing the Euler angles as
 *            `rpy = [roll, pitch, yaw]`.
 *            These are defined such that they represent the rotations about the
 *            body-fixed z-y'-x'' axes.
 * @return A Quaternion representing the same rotation given by the input Euler
 *         angles `rpy`.
 */
template <typename Derived>
Quaternion<typename Derived::Scalar> RollPitchYawToQuaternion(
    const Eigen::MatrixBase<Derived>& rpy) {
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Eigen::MatrixBase<Derived>, 3);
  return quat2eigenQuaternion(rpy2quat(rpy));
}

}  // namespace math
}  // namespace drake
