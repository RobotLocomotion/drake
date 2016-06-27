/// @file
/// Utilities for arithmetic on quaternions.

#pragma once

#include <Eigen/Dense>

namespace drake {
namespace math {

template <typename Derived>
Eigen::Matrix<typename Derived::Scalar, 4, 1> quatConjugate(
    const Eigen::MatrixBase<Derived>& q) {
  using namespace Eigen;
  static_assert(Derived::SizeAtCompileTime == 4, "Wrong size.");
  Matrix<typename Derived::Scalar, 4, 1> q_conj;
  q_conj << q(0), -q(1), -q(2), -q(3);
  return q_conj;
}

template <typename Derived1, typename Derived2>
Eigen::Matrix<typename Derived1::Scalar, 4, 1> quatProduct(
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
  Eigen::Matrix<typename Derived1::Scalar, 4, 1> r;
  r << ret_eigen.w(), ret_eigen.x(), ret_eigen.y(), ret_eigen.z();

  return r;
}

template <typename DerivedQ, typename DerivedV>
Eigen::Matrix<typename DerivedV::Scalar, 3, 1> quatRotateVec(
    const Eigen::MatrixBase<DerivedQ>& q,
    const Eigen::MatrixBase<DerivedV>& v) {
  using namespace Eigen;
  static_assert(DerivedQ::SizeAtCompileTime == 4, "Wrong size.");
  static_assert(DerivedV::SizeAtCompileTime == 3, "Wrong size.");

  typedef Matrix<typename DerivedV::Scalar, 4, 1> Vector4;
  typedef Matrix<typename DerivedV::Scalar, 3, 1> Vector3;

  Vector4 v_quat;
  v_quat << 0, v;
  auto q_times_v = quatProduct(q, v_quat);
  auto q_conj = quatConjugate(q);
  auto v_rot = quatProduct(q_times_v, q_conj);
  Vector3 r = v_rot.template bottomRows<3>();
  return r;
}

template <typename Derived1, typename Derived2>
Eigen::Matrix<typename Derived1::Scalar, 4, 1> quatDiff(
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

}  // namespace math
}  // namespace drake
