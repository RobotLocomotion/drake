/// @file
/// Utilities for arithmetic on quaternions.

#pragma once

#include <cmath>

#include <Eigen/Dense>

#include "drake/common/eigen_types.h"

#include "unsupported/Eigen/EulerAngles"

namespace drake {
namespace math {

template <typename Derived>
Vector4<typename Derived::Scalar> quatConjugate(
    const Eigen::MatrixBase<Derived>& q) {
  static_assert(Derived::SizeAtCompileTime == 4, "Wrong size.");
  Vector4<typename Derived::Scalar> q_conj;
  q_conj << q(0), -q(1), -q(2), -q(3);
  return q_conj;
}

template <typename Derived1, typename Derived2>
Vector4<typename Derived1::Scalar> quatProduct(
    const Eigen::MatrixBase<Derived1>& q1,
    const Eigen::MatrixBase<Derived2>& q2) {
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
  return quatProduct(quatConjugate(q1), q2);
}

template <typename Derived1, typename Derived2, typename DerivedU>
typename Derived1::Scalar quatDiffAxisInvar(
    const Eigen::MatrixBase<Derived1>& q1,
    const Eigen::MatrixBase<Derived2>& q2,
    const Eigen::MatrixBase<DerivedU>& u) {
  static_assert(DerivedU::SizeAtCompileTime == 3, "Wrong size.");
  auto r = quatDiff(q1, q2);
  return -2.0 + 2 * r(0) * r(0) +
         2 * pow(u(0) * r(1) + u(1) * r(2) + u(2) * r(3), 2);
}

template <typename Derived>
typename Derived::Scalar quatNorm(const Eigen::MatrixBase<Derived>& q) {
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
 * @param q a 4 x 1 vector, the unit length quaternion
 * @return [x; y; z; angle] a 4 x 1 vecotr, the angle-axis representation of a
 * rotation, the angle satisfies -PI < angle <= PI, and the axis [x;y;z]
 * has unit length.
 * The cost of this operation is roughly one atan2, one sqrt, and one divide
 * (about 100 flops)
 */
template <typename Derived>
Vector4<typename Derived::Scalar> quat2axis(
    const Eigen::MatrixBase<Derived>& q) {
  using std::sqrt;
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Eigen::MatrixBase<Derived>, 4);
  using Scalar = typename Derived::Scalar;
  Scalar sa2 = q.template tail<3>().norm();  // sin(angle/2)*axis
  Scalar epsilon_scalar = std::numeric_limits<Scalar>::epsilon();

  Vector4<Scalar> a;
  if (sa2 < epsilon_scalar * epsilon_scalar) {
    // no rotation
    a << 1.0, 0.0, 0.0, 0.0;
    return a;
  } else {
    // Use atan2.  Do NOT just use acos(q[0]) to calculate the rotation angle!!!
    // Otherwise results are numerical garbage anywhere near zero (or less
    // near).
    Scalar angle = 2 * std::atan2(sa2, q(0));

    // Since sa2>=0, atan2 returns a value between 0 and pi, which is then
    // multiplied by 2 which means the angle is between 0 and 2pi.
    // We want an angle in the range:  -pi < angle <= pi range.
    // E.g., instead of rotating 359 degrees clockwise, rotate -1 degree
    // counterclockwise.
    if (angle > M_PI) angle -= 2 * M_PI;

    // Normalize the axis part of the return value
    a.template head<3>() = q.template tail<3>() / sa2;
    a(3) = angle;
    return a;
  }
}

template <typename Derived>
Matrix3<typename Derived::Scalar> quat2rotmat(
    const Eigen::MatrixBase<Derived>& q) {
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Eigen::MatrixBase<Derived>, 4);
  auto q_normalized = q.normalized();
  auto w = q_normalized(0);
  auto x = q_normalized(1);
  auto y = q_normalized(2);
  auto z = q_normalized(3);

  Matrix3<typename Derived::Scalar> M;
  M.row(0) << w * w + x * x - y * y - z * z, 2.0 * x * y - 2.0 * w * z,
      2.0 * x * z + 2.0 * w * y;
  M.row(1) << 2.0 * x * y + 2.0 * w * z, w * w + y * y - x * x - z * z,
      2.0 * y * z - 2.0 * w * x;
  M.row(2) << 2.0 * x * z - 2.0 * w * y, 2.0 * y * z + 2.0 * w * x,
      w * w + z * z - x * x - y * y;

  return M;
}

template <typename Derived>
Vector3<typename Derived::Scalar> quat2rpy(
    const Eigen::MatrixBase<Derived>& q) {
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Eigen::MatrixBase<Derived>, 4);
  Eigen::Quaternion<typename Derived::Scalar> q_eigen(q(0), q(1), q(2), q(3));
  Eigen::Matrix<typename Derived::Scalar, 3, 3> rotmat =
      q_eigen.toRotationMatrix();
  auto euler_angles =
      Eigen::EulerAngles<typename Derived::Scalar, Eigen::EulerSystemZYX>::
          template FromRotation<false, false, false>(rotmat);
  return drake::Vector3<typename Derived::Scalar>(
      euler_angles.gamma(), euler_angles.beta(), euler_angles.alpha());
}

template <typename Derived>
Eigen::Quaternion<typename Derived::Scalar> quat2eigenQuaternion(
    const Eigen::MatrixBase<Derived>& q) {
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
  return Eigen::Quaternion<typename Derived::Scalar>(q(0), q(1), q(2), q(3));
}

}  // namespace math
}  // namespace drake
