/**
 * @file
 * This file includes conversions from roll-pitch-yaw representation,
 * that do *not* depend on quaternion.h.
 *
 * The old file roll_pitch_yaw.h was separated into two half-files, namely:
 * roll_pitch_yaw_not_using_quaternion.h and roll_pitch_yaw_using_quaternion.h,
 * so quaternion.h can include roll_pitch_yaw_not_using_quaternion.h
 * to call rpy2rotmat in quaternion.h, without circular dependency problems.
 *
 * Most users should just include roll_pitch_yaw.h (ignore the half-files).
*/
#pragma once

#include <cmath>

#include <Eigen/Dense>

#include "drake/common/drake_deprecated.h"
#include "drake/common/eigen_types.h"

namespace drake {
namespace math {

/// (Deprecated), use @ref math::RollPitchYaw(rpy).ToQuaternion().
// TODO(mitiguy) Delete this code that was deprecated on April 16, 2018.
template <typename Derived>
DRAKE_DEPRECATED("This code is deprecated per issue #8323. "
                 "Use RollPitchYaw::ToQuaternion().")
Vector4<typename Derived::Scalar> rpy2quat(
    const Eigen::MatrixBase<Derived>& rpy) {
  // TODO(hongkai.dai@tri.global): Switch to Eigen's EulerAngles when we fix
  // the range problem in Eigen
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Eigen::MatrixBase<Derived>, 3);
  auto rpy_2 = (rpy / 2.0).array();
  auto s = rpy_2.sin();
  auto c = rpy_2.cos();

  Eigen::Vector4d q;
  q << c(0) * c(1) * c(2) + s(0) * s(1) * s(2),
      s(0) * c(1) * c(2) - c(0) * s(1) * s(2),
      c(0) * s(1) * c(2) + s(0) * c(1) * s(2),
      c(0) * c(1) * s(2) - s(0) * s(1) * c(2);
  return q;
}

template <typename Derived>
Eigen::Matrix<typename Derived::Scalar, 9, 3> drpy2rotmat(
    const Eigen::MatrixBase<Derived>& rpy) {
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Eigen::MatrixBase<Derived>, 3);
  auto rpy_array = rpy.array();
  auto s = rpy_array.sin();
  auto c = rpy_array.cos();

  Eigen::Matrix<typename Derived::Scalar, 9, 3> dR;
  dR.row(0) << 0, c(2) * -s(1), c(1) * -s(2);
  dR.row(1) << 0, -s(1) * s(2), c(2) * c(1);
  dR.row(2) << 0, -c(1), 0;
  dR.row(3) << c(2) * s(1) * c(0) - s(2) * -s(0), c(2) * c(1) * s(0),
      -s(2) * s(1) * s(0) - c(2) * c(0);
  dR.row(4) << s(2) * s(1) * c(0) + c(2) * -s(0), s(2) * c(1) * s(0),
      c(2) * s(1) * s(0) - s(2) * c(0);
  dR.row(5) << c(1) * c(0), -s(1) * s(0), 0;
  dR.row(6) << c(2) * s(1) * -s(0) + s(2) * c(0), c(2) * c(1) * c(0),
      -s(2) * s(1) * c(0) + c(2) * s(0);
  dR.row(7) << s(2) * s(1) * -s(0) - c(2) * c(0), s(2) * c(1) * c(0),
      c(2) * s(1) * c(0) + s(2) * s(0);
  dR.row(8) << c(1) * -s(0), -s(1) * c(0), 0;

  return dR;
}
}  // namespace math
}  // namespace drake
