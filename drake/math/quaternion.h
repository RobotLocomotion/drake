/// @file
/// Utilities for arithmetic on quaternions.

#pragma once

#include <cmath>

#include <Eigen/Dense>

#include "drake/common/drake_assert.h"
#include "drake/common/eigen_types.h"
#include "drake/math/rotation_matrix.h"

namespace drake {
namespace math {
template <typename Derived>
Vector4<typename Derived::Scalar> quatConjugate(
    const Eigen::MatrixBase<Derived>& q) {
  // TODO(hongkai.dai@tri.global): Switch to Eigen's Quaternion when we fix
  // the range problem in Eigen
  static_assert(Derived::SizeAtCompileTime == 4, "Wrong size.");
  Vector4<typename Derived::Scalar> q_conj;
  q_conj << q(0), -q(1), -q(2), -q(3);
  return q_conj;
}

template <typename Derived1, typename Derived2>
Vector4<typename Derived1::Scalar> quatProduct(
    const Eigen::MatrixBase<Derived1>& q1,
    const Eigen::MatrixBase<Derived2>& q2) {
  // TODO(hongkai.dai@tri.global): Switch to Eigen's Quaternion when we fix
  // the range problem in Eigen
  static_assert(Derived1::SizeAtCompileTime == 4, "Wrong size.");
  static_assert(Derived2::SizeAtCompileTime == 4, "Wrong size.");

  Eigen::Quaternion<typename Derived1::Scalar> q1_eigen(q1(0), q1(1), q1(2),
                                                        q1(3));
  Eigen::Quaternion<typename Derived2::Scalar> q2_eigen(q2(0), q2(1), q2(2),
                                                        q2(3));
  auto ret_eigen = q1_eigen * q2_eigen;
  Vector4<typename Derived1::Scalar> r;
  r << ret_eigen.w(), ret_eigen.x(), ret_eigen.y(), ret_eigen.z();

  return r;
}

template <typename DerivedQ, typename DerivedV>
Vector3<typename DerivedV::Scalar> quatRotateVec(
    const Eigen::MatrixBase<DerivedQ>& q,
    const Eigen::MatrixBase<DerivedV>& v) {
  // TODO(hongkai.dai@tri.global): Switch to Eigen's Quaternion when we fix
  // the range problem in Eigen
  static_assert(DerivedQ::SizeAtCompileTime == 4, "Wrong size.");
  static_assert(DerivedV::SizeAtCompileTime == 3, "Wrong size.");

  Vector4<typename DerivedV::Scalar> v_quat;
  v_quat << 0, v;
  auto q_times_v = quatProduct(q, v_quat);
  auto q_conj = quatConjugate(q);
  auto v_rot = quatProduct(q_times_v, q_conj);
  Vector3<typename DerivedV::Scalar> r = v_rot.template bottomRows<3>();
  return r;
}

template <typename Derived1, typename Derived2>
Vector4<typename Derived1::Scalar> quatDiff(
    const Eigen::MatrixBase<Derived1>& q1,
    const Eigen::MatrixBase<Derived2>& q2) {
  // TODO(hongkai.dai@tri.global): Switch to Eigen's Quaternion when we fix
  // the range problem in Eigen
  return quatProduct(quatConjugate(q1), q2);
}

template <typename Derived1, typename Derived2, typename DerivedU>
typename Derived1::Scalar quatDiffAxisInvar(
    const Eigen::MatrixBase<Derived1>& q1,
    const Eigen::MatrixBase<Derived2>& q2,
    const Eigen::MatrixBase<DerivedU>& u) {
  // TODO(hongkai.dai@tri.global): Switch to Eigen's Quaternion when we fix
  // the range problem in Eigen
  static_assert(DerivedU::SizeAtCompileTime == 3, "Wrong size.");
  auto r = quatDiff(q1, q2);
  return -2.0 + 2 * r(0) * r(0) +
         2 * pow(u(0) * r(1) + u(1) * r(2) + u(2) * r(3), 2);
}

template <typename Derived>
typename Derived::Scalar quatNorm(const Eigen::MatrixBase<Derived>& q) {
  // TODO(hongkai.dai@tri.global): Switch to Eigen's Quaternion when we fix
  // the range problem in Eigen
  using std::acos;
  return acos(q(0));
}

/**
 * Q = Slerp(q1, q2, f) Spherical linear interpolation between two quaternions
 *   This function uses the implementation given in Algorithm 8 of [1].
 *
 * @param q1   Initial quaternion (w, x, y, z)
 * @param q2   Final quaternion (w, x, y, z)
 * @param interpolation_parameter between 0 and 1 (inclusive)
 * @retval Q   Interpolated quaternion(s). 4-by-1 vector.
 *
 * [1] Kuffner, J.J., "Effective sampling and distance metrics for 3D rigid
 * body path planning," Robotics and Automation, 2004. Proceedings. ICRA '04.
 * 2004 IEEE International Conference on , vol.4, no., pp.3993, 3998 Vol.4,
 * April 26-May 1, 2004
 * doi: 10.1109/ROBOT.2004.1308895
 */
template <typename Derived1, typename Derived2, typename Scalar>
Vector4<Scalar> Slerp(const Eigen::MatrixBase<Derived1>& q1,
                      const Eigen::MatrixBase<Derived2>& q2,
                      const Scalar& interpolation_parameter) {
  // TODO(hongkai.dai@tri.global): Switch to Eigen's Quaternion when we fix
  // the range problem in Eigen
  using std::acos;
  using std::sin;

  // Compute the quaternion inner product
  auto lambda = (q1.transpose() * q2).value();
  int q2_sign;
  if (lambda < Scalar(0)) {
    // The quaternions are pointing in opposite directions, so use the
    // equivalent alternative representation for q2
    lambda = -lambda;
    q2_sign = -1;
  } else {
    q2_sign = 1;
  }

  // Calculate interpolation factors
  // TODO(tkoolen): do we really want an epsilon so small?
  Scalar r, s;
  if (std::abs(1.0 - lambda) < Eigen::NumTraits<Scalar>::epsilon()) {
    // The quaternions are nearly parallel, so use linear interpolation
    r = 1.0 - interpolation_parameter;
    s = interpolation_parameter;
  } else {
    Scalar alpha = acos(lambda);
    Scalar gamma = 1.0 / sin(alpha);
    r = std::sin((1.0 - interpolation_parameter) * alpha) * gamma;
    s = std::sin(interpolation_parameter * alpha) * gamma;
  }

  auto ret = (q1 * r).eval();
  ret += q2 * (q2_sign * s);
  return ret;
}

/** Adapts the code from simbody
 * https://github.com/simbody/simbody/blob/master/SimTKcommon/Mechanics/src/Quaternion.cpp
 * @param quaternion a 4 x 1 vector, the quaternion that has been normalized to
 * unit length.
 * @return [x; y; z; angle] a 4 x 1 vector, the axis-angle representation of a
 * rotation, the angle satisfies -PI < angle <= PI, and the axis [x;y;z]
 * has unit length.
 * The cost of this function is roughly one atan2, one sqrt, and one divide
 * (about 100 flops)
 */
template <typename Derived>
Vector4<typename Derived::Scalar> quat2axis(
    const Eigen::MatrixBase<Derived>& quaternion) {
  // TODO(hongkai.dai@tri.global): Switch to Eigen's Quaternion when we fix
  // the range problem in Eigen
  using std::sqrt;
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Eigen::MatrixBase<Derived>, 4);
  using Scalar = typename Derived::Scalar;
  Scalar abs_sin_half_angle =
      quaternion.template tail<3>().norm();  // abs(sin(angle/2))
  Scalar epsilon_scalar = Eigen::NumTraits<Scalar>::epsilon();

  Vector4<Scalar> axis_angle;
  if (abs_sin_half_angle < epsilon_scalar * epsilon_scalar) {
    // No rotation - arbitrarily return x-axis rotation of 0 degrees.
    axis_angle << 1.0, 0.0, 0.0, 0.0;
    return axis_angle;
  } else {
    // Use atan2.  Do NOT just use acos(q[0]) to calculate the rotation angle!!!
    // Otherwise results are numerical garbage anywhere where abs_sin_half_angle
    // (or equivalent rotation angle) is close to zero.
    Scalar angle = 2 * std::atan2(abs_sin_half_angle, quaternion(0));

    // Since sa2 >= 0, atan2 returns a value between 0 and pi, which is then
    // multiplied by 2 which means the angle is between 0 and 2pi.
    // We want an angle in the range:  -pi < angle <= pi range.
    // E.g., instead of rotating 359 degrees clockwise, rotate -1 degree
    // counterclockwise.
    if (angle > M_PI) angle -= 2 * M_PI;

    // Normalize the axis part of the return value.
    axis_angle.template head<3>() =
        quaternion.template tail<3>() / abs_sin_half_angle;
    axis_angle(3) = angle;
    return axis_angle;
  }
}

/**
 * Computes the rotation matrix from quaternion representation.
 * @param quaternion A 4 x 1 unit length quaternion, @p q=[w;x;y;z]
 * @return A 3 x 3 rotation matrix
 */
template <typename Derived>
Matrix3<typename Derived::Scalar> quat2rotmat(
    const Eigen::MatrixBase<Derived>& quaternion) {
  // TODO(hongkai.dai@tri.global): Switch to Eigen's Quaternion when we fix
  // the range problem in Eigen
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Eigen::MatrixBase<Derived>, 4);
  auto q_normalized = quaternion.normalized();
  auto w = q_normalized(0);
  auto x = q_normalized(1);
  auto y = q_normalized(2);
  auto z = q_normalized(3);

  auto ww = w * w;
  auto xx = x * x;
  auto yy = y * y;
  auto zz = z * z;
  auto wx = w * x;
  auto wy = w * y;
  auto wz = w * z;
  auto xy = x * y;
  auto xz = x * z;
  auto yz = y * z;
  Matrix3<typename Derived::Scalar> M;
  M.row(0) << ww + xx - yy - zz, 2.0 * xy - 2.0 * wz, 2.0 * xz + 2.0 * wy;
  M.row(1) << 2.0 * xy + 2.0 * wz, ww + yy - xx - zz, 2.0 * yz - 2.0 * wx;
  M.row(2) << 2.0 * xz - 2.0 * wy, 2.0 * yz + 2.0 * wx, ww + zz - xx - yy;

  return M;
}

/**
 * Computes the Euler angles from quaternion representation.
 * @param quaternion A 4 x 1 unit length vector @p q=[w;x;y;z]
 * @return A 3 x 1 Euler angles about Body-fixed z-y'-x'' axes by [rpy(2),
 * rpy(1), rpy(0)]
 * @see rpy2rotmat
 * When the pitch angle is close to PI/2 or -PI/2, this function is not very
 * accurate. For pitch = PI/2 - 1E-6, the error can be in the order of 1E-7.
 * The error gets larger when the pitch gets closer to PI/2 or -PI/2.
 */
template <typename Derived>
Vector3<typename Derived::Scalar> quat2rpy(
    const Eigen::MatrixBase<Derived>& quaternion) {
  // TODO(hongkai.dai@tri.global): Switch to Eigen's Quaternion when we fix
  // the range problem in Eigen
  // TODO(mitiguy@tri.global): replace this method with the high-precision
  // method
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Eigen::MatrixBase<Derived>, 4);
  return rotmat2rpy(quat2rotmat(quaternion));
}

// The Eigen Quaterniond constructor when used with 4 arguments, uses the (w,
// x, y, z) ordering, just as we do.
// HOWEVER: when the constructor is called on a 4-element Vector, the elements
// must be in (x, y, z, w) order.
// So, the following two calls will give you the SAME quaternion:
// Quaternion<double>(q(0), q(1), q(2), q(3));
// Quaternion<double>(Vector4d(q(3), q(0), q(1), q(2)))
// which is gross and will cause you much pain.
// see:
// http://eigen.tuxfamily.org/dox/classEigen_1_1Quaternion.html#a91b6ea2cac13ab2d33b6e74818ee1490
//
// This method takes a nice, normal (w, x, y, z) order vector and gives you
// the Quaternion you expect.
template <typename Derived>
Eigen::Quaternion<typename Derived::Scalar> quat2eigenQuaternion(
    const Eigen::MatrixBase<Derived>& q) {
  // TODO(hongkai.dai@tri.global): Switch to Eigen's Quaternion when we fix
  // the range problem in Eigen
  return Eigen::Quaternion<typename Derived::Scalar>(q(0), q(1), q(2), q(3));
}

}  // namespace math
}  // namespace drake
