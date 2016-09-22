/// @file
/// Utilities for arithmetic on axis-angle rotations.

#pragma once

#include <cmath>

#include <Eigen/Dense>

#include "drake/common/eigen_types.h"
#include "drake/math/rotation_matrix.h"

namespace drake {
namespace math {
template <typename Derived>
Eigen::AngleAxis<typename Derived::Scalar> axisToEigenAngleAxis(const Eigen::MatrixBase<Derived> &a) {
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Eigen::MatrixBase<Derived>, 4);
  return Eigen::AngleAxis<typename Derived::Scalar>(a(3), a.template head<3>());
}

template <typename Derived>
Vector4<typename Derived::Scalar> axis2quat(
    const Eigen::MatrixBase<Derived>& a) {
  using std::cos;
  using std::sin;
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Eigen::MatrixBase<Derived>, 4);
  auto axis = a.template head<3>();
  auto angle = a(3);
  auto arg = 0.5 * angle;
  auto c = cos(arg);
  auto s = sin(arg);
  Vector4<typename Derived::Scalar> ret;
  ret << c, s * axis;
  return ret;
}

template <typename Derived>
Matrix3<typename Derived::Scalar> axis2rotmat(
    const Eigen::MatrixBase<Derived>& a) {
  return axisToEigenAngleAxis(a).toRotationMatrix();
}

template <typename Derived>
Vector3<typename Derived::Scalar> axis2rpy(
    const Eigen::MatrixBase<Derived>& a) {
  return rotmat2rpy(axis2rotmat(a));
}

}  // namespace math
}  // namespace drake
