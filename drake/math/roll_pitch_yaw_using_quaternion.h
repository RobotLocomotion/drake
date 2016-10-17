// @file This file includes the conversion from a roll pitch yaw representation,
// that does use quaternion.h
// We break roll_pitch_yaw.h into two half-files
//      roll_pitch_yaw_no_using_quaternion.h
// and  roll_pitch_yaw_using_quaternion.h,
// so that quaternion.h can include roll_pitch_yaw_no_using_quaternion.h
// to call rpy2rotmat in quaternion.h, without causing circular dependency
// problem.
// Most users should include roll_pitch_yaw.h, and ignore the half-files
// roll_pitch_yaw_no_using_quaternion.h and roll_pitch_yaw_using_quaternion.h
#pragma once

#include "drake/math/quaternion.h"
#include "drake/math/roll_pitch_yaw_no_using_quaternion.h"

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
}  // namespace math
}  // namespace drake
