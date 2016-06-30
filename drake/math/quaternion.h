/// @file
/// Utilities for arithmetic on quaternions.

#pragma once

#include <cmath>

#include <Eigen/Dense>

#include "drake/common/eigen_types.h"

namespace drake {
namespace math {

template <typename Derived>
Vector4<typename Derived::Scalar> quatConjugate(
    const Eigen::MatrixBase<Derived>& q) {
  using namespace Eigen;
  static_assert(Derived::SizeAtCompileTime == 4, "Wrong size.");
  Vector4<typename Derived::Scalar> q_conj;
  q_conj << q(0), -q(1), -q(2), -q(3);
  return q_conj;
}

template <typename Derived1, typename Derived2>
Vector4<typename Derived1::Scalar> quatProduct(
    const Eigen::MatrixBase<Derived1>& q1,
    const Eigen::MatrixBase<Derived2>& q2) {
  using namespace Eigen;
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
  using namespace Eigen;
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
  return std::acos(q(0));
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
    Scalar alpha = std::acos(lambda);
    Scalar gamma = 1.0 / std::sin(alpha);
    r = std::sin((1.0 - interpolation_parameter) * alpha) * gamma;
    s = std::sin(interpolation_parameter * alpha) * gamma;
  }

  auto ret = (q1 * r).eval();
  ret += q2 * (q2_sign * s);
  return ret;
}

}  // namespace math
}  // namespace drake
