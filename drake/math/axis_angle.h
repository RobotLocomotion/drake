/// @file
/// Utilities for arithmetic on axis-angle rotations.

#pragma once

#include <cmath>

#include <Eigen/Dense>

#include "drake/common/eigen_types.h"
#include "drake/math/quaternion.h"

namespace drake {
namespace math {
template <typename Derived>
Eigen::Vector4d axis2quat(const Eigen::MatrixBase<Derived>& a) {
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Eigen::MatrixBase<Derived>, 4);
  auto axis = a.template head<3>();
  auto angle = a(3);
  auto arg = 0.5 * angle;
  auto c = std::cos(arg);
  auto s = std::sin(arg);
  Eigen::Vector4d ret;
  ret << c, s * axis;
  return ret;
}

template <typename Derived>
Eigen::Matrix<typename Derived::Scalar, 3, 3> axis2rotmat(
    const Eigen::MatrixBase<Derived>& a) {
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Eigen::MatrixBase<Derived>, 4);
  const auto& axis = (a.template head<3>()) / (a.template head<3>()).norm();
  const auto& theta = a(3);
  auto x = axis(0);
  auto y = axis(1);
  auto z = axis(2);
  auto ctheta = std::cos(theta);
  auto stheta = std::sin(theta);
  auto c = 1 - ctheta;
  Eigen::Matrix<typename Derived::Scalar, 3, 3> R;
  R << ctheta + x * x * c, x * y * c - z * stheta, x * z * c + y * stheta,
      y * x * c + z * stheta, ctheta + y * y * c, y * z * c - x * stheta,
      z * x * c - y * stheta, z * y * c + x * stheta, ctheta + z * z * c;

  return R;
}

template <typename Derived>
Eigen::Matrix<typename Derived::Scalar, 3, 1> axis2rpy(
    const Eigen::MatrixBase<Derived>& a) {
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Eigen::MatrixBase<Derived>, 4);
  return quat2rpy(axis2quat(a));
}

}  // namespace math
}  // namespace drake
