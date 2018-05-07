/// @file
/// Utilities for arithmetic on quaternions.

#pragma once

#include <cmath>

#include <Eigen/Dense>

#include "drake/common/drake_assert.h"
#include "drake/common/eigen_types.h"
#include "drake/common/is_approx_equal_abstol.h"
#include "drake/math/rotation_matrix.h"

namespace drake {
namespace math {

// Eigen's 4-argument Quaternion constructor uses (w, x, y, z) ordering.
// HOWEVER: If you use Eigen's 1-argument Quaternion constructor, where the one
// argument is a 4-element Vector, the elements must be in (x, y, z, w) order!
// So, the following two calls will give you the SAME quaternion:
// Quaternion<double>(q(0), q(1), q(2), q(3));
// Quaternion<double>(Vector4d(q(3), q(0), q(1), q(2)))
// which is gross and will cause you much pain.  See:
// http://eigen.tuxfamily.org/dox/classEigen_1_1Quaternion.html#a91b6ea2cac13ab2d33b6e74818ee1490
//
// This method takes a nice, normal (w, x, y, z) order vector and gives you
// the Quaternion you expect.
// (Deprecated), use @ref Eigen::Quaternion(w, x, y, z).
// TODO(mitiguy) Delete this code that was deprecated on May 1, 2018.
template <typename Derived>
DRAKE_DEPRECATED("This code is deprecated.  Use Eigen's Quaternion constructor"
                 "Quaternion(w, x, y, z) -- not a home-brew 4-element vectors")
Eigen::Quaternion<typename Derived::Scalar> quat2eigenQuaternion(
    const Eigen::MatrixBase<Derived>& q) {
  return Eigen::Quaternion<typename Derived::Scalar>(q(0), q(1), q(2), q(3));
}

/**
 * Returns a unit quaternion that represents the same orientation as `q1`,
 * and has the "shortest" geodesic distance on the unit sphere to `q0`.
 */
template <typename Scalar> Eigen::Quaternion<Scalar> ClosestQuaternion(
    const Eigen::Quaternion<Scalar>& q0,
    const Eigen::Quaternion<Scalar>& q1) {
  Eigen::Quaternion<Scalar> q = q1;
  if (q0.dot(q) < 0)
    q.coeffs() *= -1;
  q.normalize();
  return q;
}

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

/**
 * Computes the rotation matrix from quaternion representation.
 * @tparam Derived An Eigen derived type, e.g., an Eigen Vector3d.
 * @param quaternion 4 x 1 unit length quaternion, @p q=[w;x;y;z]
 * @return 3 x 3 rotation matrix
 * (Deprecated), use @ref math::RotationMatrix(quaternion).
 */
// TODO(mitiguy) change all calling sites to this function.
template <typename Derived>
Matrix3<typename Derived::Scalar> quat2rotmat(
    const Eigen::MatrixBase<Derived>& v) {
  const Eigen::Quaternion<typename Derived::Scalar> q(v(0), v(1), v(2), v(3));
  const RotationMatrix<typename Derived::Scalar> R(q);
  return R.matrix();
}

/**
 * This function tests whether a quaternion is in "canonical form" meaning that
 * it tests whether the quaternion [w, x, y, z] has a non-negative w value.
 * Example: [-0.3, +0.4, +0.5, +0.707] is not in canonical form.
 * Example: [+0.3, -0.4, -0.5, -0.707] is in canonical form.
 * @param quat Quaternion [w, x, y, z] that relates two right-handed
 *   orthogonal unitary bases e.g., Ax, Ay, Az (A) to Bx, By, Bz (B).
 *   Note: quat is analogous to the rotation matrix R_AB.
 * @return `true` if quat.w() is nonnegative (in canonical form), else `false`.
 */
template<typename T>
bool is_quaternion_in_canonical_form(const Eigen::Quaternion<T>& quat) {
  return quat.w() >= 0.0;
}


/**
 * This function returns a quaternion in its "canonical form" meaning that
 * it returns a quaternion [w, x, y, z] with a non-negative w.
 * For example, if passed a quaternion [-0.3, +0.4, +0.5, +0.707], the function
 * returns the quaternion's canonical form [+0.3, -0.4, -0.5, -0.707].
 * @param quat Quaternion [w, x, y, z] that relates two right-handed
 *   orthogonal unitary bases e.g., Ax, Ay, Az (A) to Bx, By, Bz (B).
 *   Note: quat is analogous to the rotation matrix R_AB.
 * @return Canonical form of quat, which means that either the original quat
 *   is returned or a quaternion representing the same orientation but with
 *   negated [w, x, y, z], to ensure a positive w in returned quaternion.
 */
template<typename T>
Eigen::Quaternion<T> QuaternionToCanonicalForm(
    const Eigen::Quaternion<T>& quat ) {
  return is_quaternion_in_canonical_form(quat) ? quat :
         Eigen::Quaternion<T>(-quat.w(), -quat.x(), -quat.y(), -quat.z());
}


/**
 * This function tests whether two quaternions represent the same orientation.
 * This function converts each quaternion to its canonical form and tests
 * whether the absolute value of the difference in corresponding elements of
 * these canonical quaternions is within tolerance.
 * @param quat1 Quaternion [w, x, y, z] that relates two right-handed
 *   orthogonal unitary bases e.g., Ax, Ay, Az (A) to Bx, By, Bz (B).
 *   Note: quat is analogous to the rotation matrix R_AB.
 * @param quat2 Quaternion with a description analogous to quat1.
 * @param tolerance Nonnegative real scalar defining the allowable difference
 *   in the orientation described by quat1 and quat2.
 * @return `true` if quat1 and quat2 represent the same orientation (to within
 * tolerance), otherwise `false`.
 */
template<typename T>
bool AreQuaternionsEqualForOrientation(
    const Eigen::Quaternion<T>& quat1,
    const Eigen::Quaternion<T>& quat2,
    const T tolerance) {
  const Eigen::Quaternion<T> quat1_canonical = QuaternionToCanonicalForm(quat1);
  const Eigen::Quaternion<T> quat2_canonical = QuaternionToCanonicalForm(quat2);
  return quat1_canonical.isApprox(quat2_canonical, tolerance);
}


/** This function calculates a quaternion's time-derivative from its quaternion
 * and angular velocity. Algorithm from [Kane, 1983] Section 1.13, Pages 58-59.
 *
 * - [Kane, 1983] "Spacecraft Dynamics," McGraw-Hill Book Co., New York, 1983.
 *   (With P. W. Likins and D. A. Levinson).  Available for free .pdf download:
 *   https://ecommons.cornell.edu/handle/1813/637
 *
 * @param quat_AB Quaternion [w, x, y, z] that relates two right-handed
 *   orthogonal unitary bases e.g., Ax, Ay, Az (A) to Bx, By, Bz (B).
 *   Note: quat_AB is analogous to the rotation matrix R_AB.
 * @param w_AB_B  B's angular velocity in A, expressed in B.
 * @retval quatDt Time-derivative of quat_AB, i.e., [ẇ, ẋ, ẏ, ż].
 */
// Note: To avoid dependence on Eigen's internal ordering of elements in its
// Quaternion class, herein we use `e0 = quat.w()', `e1 = quat.x()`, etc.
// Return value `quatDt` *does* have a specific order as defined above.
template<typename T>
Vector4<T> CalculateQuaternionDtFromAngularVelocityExpressedInB(
    const Eigen::Quaternion<T>& quat_AB,  const Vector3<T>& w_AB_B ) {
  const T e0 = quat_AB.w(),  e1 = quat_AB.x(),
      e2 = quat_AB.y(),  e3 = quat_AB.z();
  const T wx = w_AB_B[0], wy = w_AB_B[1], wz = w_AB_B[2];

  const T e0Dt = (-e1*wx - e2*wy - e3*wz) / 2;
  const T e1Dt =  (e0*wx - e3*wy + e2*wz) / 2;
  const T e2Dt =  (e3*wx + e0*wy - e1*wz) / 2;
  const T e3Dt = (-e2*wx + e1*wy + e0*wz) / 2;

  return Vector4<T>(e0Dt, e1Dt, e2Dt, e3Dt);
}


/** This function calculates angular velocity from a quaternion and its time-
 * derivative. Algorithm from [Kane, 1983] Section 1.13, Pages 58-59.
 *
 * - [Kane, 1983] "Spacecraft Dynamics," McGraw-Hill Book Co., New York, 1983.
 *   (with P. W. Likins and D. A. Levinson).  Available for free .pdf download:
 *   https://ecommons.cornell.edu/handle/1813/637
 *
 * @param quat_AB  Quaternion [w, x, y, z] that relates two right-handed
 *   orthogonal unitary bases e.g., Ax, Ay, Az (A) to Bx, By, Bz (B).
 *   Note: quat_AB is analogous to the rotation matrix R_AB.
 * @param quatDt  Time-derivative of `quat_AB`, i.e. [ẇ, ẋ, ẏ, ż].
 * @retval w_AB_B  B's angular velocity in A, expressed in B.
 */
// Note: To avoid dependence on Eigen's internal ordering of elements in its
// Quaternion class, herein we use `e0 = quat.w()', `e1 = quat.x()`, etc.
// Parameter `quatDt` *does* have a specific order as defined above.
template <typename T>
Vector3<T> CalculateAngularVelocityExpressedInBFromQuaternionDt(
    const Eigen::Quaternion<T>& quat_AB, const Vector4<T>& quatDt) {
  const T e0 = quat_AB.w(), e1 = quat_AB.x(),
      e2 = quat_AB.y(), e3 = quat_AB.z();
  const T e0Dt = quatDt[0], e1Dt = quatDt[1],
      e2Dt = quatDt[2], e3Dt = quatDt[3];

  const T wx = 2*(-e1*e0Dt + e0*e1Dt + e3*e2Dt - e2*e3Dt);
  const T wy = 2*(-e2*e0Dt - e3*e1Dt + e0*e2Dt + e1*e3Dt);
  const T wz = 2*(-e3*e0Dt + e2*e1Dt - e1*e2Dt + e0*e3Dt);

  return Vector3<T>(wx, wy, wz);
}


/** This function calculates how well a quaternion and its time-derivative
 * satisfy the quaternion time-derivative constraint specified in [Kane, 1983]
 * Section 1.13, equations 12-13, page 59.  For a quaternion [w, x, y, z],
 * the quaternion must satisfy:  w^2 + x^2 + y^2 + z^2 = 1,   hence its
 * time-derivative must satisfy:  2*(w*ẇ + x*ẋ + y*ẏ + z*ż) = 0.
 *
 * - [Kane, 1983] "Spacecraft Dynamics," McGraw-Hill Book Co., New York, 1983.
 *   (with P. W. Likins and D. A. Levinson).  Available for free .pdf download:
 *   https://ecommons.cornell.edu/handle/1813/637
 *
 * @param quat  Quaternion [w, x, y, z] that relates two right-handed
 *   orthogonal unitary bases e.g., Ax, Ay, Az (A) to Bx, By, Bz (B).
 *   Note: A quaternion like quat_AB is analogous to the rotation matrix R_AB.
 * @param quatDt  Time-derivative of `quat`, i.e., [ẇ, ẋ, ẏ, ż].
 * @retval quaternionDt_constraint_violation  The amount the time-
 *   derivative of the quaternion constraint has been violated, which may be
 *   positive or negative (0 means the constraint is perfectly satisfied).
 */
template <typename T>
T CalculateQuaternionDtConstraintViolation(const Eigen::Quaternion<T>& quat,
                                           const Vector4<T>& quatDt) {
  const T w = quat.w(), x = quat.x(), y = quat.y(), z = quat.z();
  const T wDt = quatDt[0], xDt = quatDt[1],  yDt = quatDt[2], zDt = quatDt[3];
  const T quaternionDt_constraint_violation = 2*(w*wDt + x*xDt + y*yDt + z*zDt);
  return quaternionDt_constraint_violation;
}

/** This function tests if a quaternion satisfies the quaternion constraint
 * specified in [Kane, 1983] Section 1.3, equation 4, page 12, i.e., a
 * quaternion [w, x, y, z] must satisfy:  w^2 + x^2 + y^2 + z^2 = 1.
 *
 * - [Kane, 1983] "Spacecraft Dynamics," McGraw-Hill Book Co., New York, 1983.
 *   (with P. W. Likins and D. A. Levinson).  Available for free .pdf download:
 *   https://ecommons.cornell.edu/handle/1813/637
 *
 * @param quat  Quaternion [w, x, y, z] that relates two right-handed
 *   orthogonal unitary bases e.g., Ax, Ay, Az (A) to Bx, By, Bz (B).
 *   Note: A quaternion like quat_AB is analogous to the rotation matrix R_AB.
 * @param tolerance  Tolerance for quaternion constraint, i.e., how much is
 *   w^2 + x^2 + y^2 + z^2  allowed to differ from 1.
 * @return `true` if the quaternion constraint is satisfied within tolerance.
 */
template <typename T>
bool IsQuaternionValid(const Eigen::Quaternion<T>& quat,
                       const double tolerance) {
  using std::abs;
  const T quat_norm_error = abs(1.0 - quat.norm());
  return (quat_norm_error <= tolerance);
}


/** This function tests if a quaternion satisfies the time-derivative constraint
 * specified in [Kane, 1983] Section 1.13, equation 13, page 59.  A quaternion
 * [w, x, y, z] must satisfy  w^2 + x^2 + y^2 + z^2 = 1,   hence its
 * time-derivative must satisfy  2*(w*ẇ + x*ẋ + y*ẏ + z*ż) = 0.
 * Note: To accurately test whether the time-derivative quaternion constraint
 * is satisfied, the quaternion constraint is also tested to be accurate.
 *
 * - [Kane, 1983] "Spacecraft Dynamics," McGraw-Hill Book Co., New York, 1983.
 *   (with P. W. Likins and D. A. Levinson).  Available for free .pdf download:
 *   https://ecommons.cornell.edu/handle/1813/637
 *
 * @param quat  Quaternion [w, x, y, z] that relates two right-handed
 *   orthogonal unitary bases e.g., Ax, Ay, Az (A) to Bx, By, Bz (B).
 *   Note: A quaternion like quat_AB is analogous to the rotation matrix R_AB.
 * @param quatDt  Time-derivative of `quat`, i.e., [ẇ, ẋ, ẏ, ż].
 * @param tolerance  Tolerance for quaternion constraints.
 * @return `true` if both of the two previous constraints are within tolerance.
 */
template <typename T>
bool IsBothQuaternionAndQuaternionDtOK(const Eigen::Quaternion<T>& quat,
                                       const Vector4<T>& quatDt,
                                       const double tolerance) {
  using std::abs;

  // For an accurate test, the quaternion should be reasonably accurate.
  if ( !IsQuaternionValid(quat, tolerance) ) return false;

  const T quatDt_test = CalculateQuaternionDtConstraintViolation(quat, quatDt);
  return abs(quatDt_test) <= tolerance;
}


/** This function tests if a quaternion and a quaternions time-derivative
 * can calculate and match an angular velocity to within a tolerance.
 * Note: This function first tests if the quaternion [w, x, y, z] satisifies
 * w^2 + x^2 + y^2 + z^2 = 1 (to within tolerance) and if its time-derivative
 * satisfies  w*ẇ + x*ẋ + y*ẏ + z*ż = 0  (to within tolerance).  Lastly, it
 * tests if each element of the angular velocity calculated from quat and quatDt
 * is within tolerance of w_B (described below).
 * @param quat  Quaternion [w, x, y, z] that relates two right-handed
 *   orthogonal unitary bases e.g., Ax, Ay, Az (A) to Bx, By, Bz (B).
 *   Note: A quaternion like quat_AB is analogous to the rotation matrix R_AB.
 * @param quatDt  Time-derivative of `quat`, i.e., [ẇ, ẋ, ẏ, ż].
 * @param w_B  Rigid body B's angular velocity in frame A, expressed in B.
 * @param tolerance  Tolerance for quaternion constraints.
 * @return `true` if all three of the previous constraints are within tolerance.
 */
template <typename T>
bool IsQuaternionAndQuaternionDtEqualAngularVelocityExpressedInB(
                                       const Eigen::Quaternion<T>& quat,
                                       const Vector4<T>& quatDt,
                                       const Vector3<T>& w_B,
                                       const double tolerance) {
  // Ensure time-derivative of quaternion satifies quarternionDt test.
  if ( !math::IsBothQuaternionAndQuaternionDtOK(quat, quatDt, tolerance) )
    return false;

  const Eigen::Vector3d w_from_quatDt =
       math::CalculateAngularVelocityExpressedInBFromQuaternionDt(quat, quatDt);
  return is_approx_equal_abstol(w_from_quatDt, w_B, tolerance);
}

}  // namespace math
}  // namespace drake
