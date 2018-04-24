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

#include "drake/common/drake_deprecated.h"
#include "drake/math/quaternion.h"
#include "drake/math/roll_pitch_yaw_not_using_quaternion.h"

namespace drake {
namespace math {

/// (Deprecated), use @ref math::RollPitchYaw(rpy).ToQuaternion().
// TODO(mitiguy) Delete this code that was deprecated on April 16, 2018.
template <typename Derived>
DRAKE_DEPRECATED("This code is deprecated per issue #8323. "
                     "Use RollPitchYaw::ToQuaternion().")
Quaternion<typename Derived::Scalar> RollPitchYawToQuaternion(
        const Eigen::MatrixBase<Derived>& rpy) {
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Eigen::MatrixBase<Derived>, 3);
  return quat2eigenQuaternion(rpy2quat(rpy));
}
}  // namespace math
}  // namespace drake
