#pragma once

#include <cmath>

#include <Eigen/Dense>

#include "drake/common/eigen_types.h"
#include "drake/math/quaternion.h"

namespace drake {
namespace math {
/**
 * Computes the quaternion representation from Euler angles.
 * @param rpy A 3 x 1 vector. The Euler angles about Body-fixed z-y'-x'' axes
 * by angles [rpy(2), rpy(1), rpy(0)].
 * @return A 4 x 1 unit length quaternion @p a=[w;x;y;z]
 * @see rpy2rotmat
 */
template <typename Derived>
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
 * We use an extrinsic rotation about Space-fixed x-y-z axes by angles [rpy(0),
 * rpy(1), rpy(2)].
 * Or equivalently, we use an intrinsic
 * rotation about Body-fixed z-y'-x'' axes by angles [rpy(2), rpy(1), rpy(0)].
 * The rotation matrix returned is equivalent to
 * rotz(rpy(2)) * roty(rpy(1)) * rotx(rpy(0)), where
 * @f[
 * rotz(a) = \begin{bmatrix} cos(a)& -sin(a) & 0\\
 *                           sin(a) & cos(a) & 0\\
 *                             0    &  0     & 1 \end{bmatrix}\;,
 * roty(b) = \begin{bmatrix} cos(b)  & 0 & sin(b)\\
 *                           0       & 1 &    0   \\
 *                           -sin(b) & 0 & cos(b)\end{bmatrix}\;,
 * rotx(c) = \begin{bmatrix} 1 &  0     &      0 \\
 *                           0 & cos(c) & -sin(c)\\
 *                           0 & sin(c) & cos(c)\end{bmatrix}
 * @f]
 */
template <typename Derived>
Matrix3<typename Derived::Scalar> rpy2rotmat(
    const Eigen::MatrixBase<Derived>& rpy) {
  // TODO(hongkai.dai@tri.global): Switch to Eigen's EulerAngles when we fix
  // the range problem in Eigen
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Eigen::MatrixBase<Derived>, 3);
  auto rpy_array = rpy.array();
  auto s = rpy_array.sin();
  auto c = rpy_array.cos();

  Matrix3<typename Derived::Scalar> R;
  R.row(0) << c(2) * c(1), c(2) * s(1) * s(0) - s(2) * c(0),
      c(2) * s(1) * c(0) + s(2) * s(0);
  R.row(1) << s(2) * c(1), s(2) * s(1) * s(0) + c(2) * c(0),
      s(2) * s(1) * c(0) - c(2) * s(0);
  R.row(2) << -s(1), c(1) * s(0), c(1) * c(0);

  return R;
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
