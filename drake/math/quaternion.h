/// @file
/// Utilities for arithmetic on quaternions.

#pragma once

#include <cmath>

#include <Eigen/Dense>

#include "drake/common/drake_assert.h"
#include "drake/common/eigen_types.h"
#include "drake/math/roll_pitch_yaw_no_using_quaternion.h"
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

/**
 * @param quaternion a 4 x 1 vector that may or may not be normalized.
 * @param is_angle_returned_in_range_0_to_2pi is a boolean.
 * @return [x; y; z; angle] a 4 x 1 vector, the axis-angle representation of a
 * rotation.  The axis is a unit vector [x;y;z].  Depending on the passed-in
 * boolean, the angle is either: -PI <= angle <= PI or 0 <= angle <= 2*PI.
 */
template <typename Derived>
Vector4<typename Derived::Scalar> QuaternionToAxisAngle(
    const Eigen::MatrixBase<Derived>& quaternion,
    const bool is_angle_returned_in_range_0_to_2pi) {
  // TODO(hongkai.dai@tri.global): Switch to Eigen's Quaternion when we fix
  // the range problem in Eigen
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Eigen::MatrixBase<Derived>, 4);
  using Scalar = typename Derived::Scalar;
  Eigen::Quaternion<Scalar> eigen_quaternion(quaternion(0), quaternion(1),
                                             quaternion(2), quaternion(3));

  // Use Eigen's built-in algorithm which seems robust (checked by Mitiguy/Dai).
  Eigen::AngleAxis<Scalar> eigen_angle_axis(eigen_quaternion);
  Scalar angle = eigen_angle_axis.angle();
  const Vector3<Scalar> axis = eigen_angle_axis.axis();

  // Decide range of return value for angle [0 to 2*PI] or [-PI to PI].
  if (!is_angle_returned_in_range_0_to_2pi && angle > M_PI) angle -= 2 * M_PI;

  // Switch from Eigen's [angle, x,y,z] order to Drake's [x,y,z, angle] order.
  Vector4<Scalar> axis_angle;
  axis_angle(3) = angle;                 // Drake's final argument is an angle.
  axis_angle.template head<3>() = axis;  // Drake's first 3 arguments are axis.

  return axis_angle;
}

/** (Deprecated) Computes Drake axis-angle from quaternion.
Use `QuaternionToAxisAngle()` instead.
@see QuaternionToAxisAngle() **/
template <typename Derived>
Vector4<typename Derived::Scalar> quat2axis(
    const Eigen::MatrixBase<Derived>& quaternion) {
  return QuaternionToAxisAngle(quaternion, true);
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
 * Computes SpaceXYZ Euler angles from quaternion representation.
 * @param quaternion 4x1 unit length vector with elements [ e0, e1, e2, e3 ].
 * @return 3x1 SpaceXYZ Euler angles (called roll-pitch-yaw by ROS).
 * Note: SpaceXYZ roll-pitch-yaw is equivalent to BodyZYX yaw-pitch-roll.
 *
http://answers.ros.org/question/58863/incorrect-rollpitch-yaw-values-using-getrpy/
 * This accurate algorithm avoids numerical round-off issues encountered by some
 * algorithms when pitch angle is within 1E-6 of PI/2 or -PI/2.
 *
 * <h3>Theory</h3>
 * This algorithm was created September 2016 by Paul Mitiguy for TRI (Toyota).
 * Notation: Angles q1, q2, q3 designate roll, pitch, and yaw.
 *           Symbols e0, e1, e2, e3 are elements of the passed-in quaternion.
 *
 * Step 1.  Convert the quaternion to a 3x3 rotation matrix R.
 *
 * Step 2.  Calculate the pitch angle q2 using the atan2 function and several
 *          elements of what is interpreted as a SpaceXYZ rotation matrix R.
 *
 * Step 3.  Realize the quaternion passed to the function can be regarded as
 *          resulting from multiplication of 4x4 and 4x1 Euler matrices to give:
 * e0 = sin(0.5*q1)*sin(0.5*q2)*sin(0.5*q3) +
cos(0.5*q1)*cos(0.5*q2)*cos(0.5*q3)
 * e1 = sin(0.5*q3)*cos(0.5*q1)*cos(0.5*q2) -
sin(0.5*q1)*sin(0.5*q2)*cos(0.5*q3)
 * e2 = sin(0.5*q1)*sin(0.5*q3)*cos(0.5*q2) +
sin(0.5*q2)*cos(0.5*q1)*cos(0.5*q3)
 * e3 = sin(0.5*q1)*cos(0.5*q2)*cos(0.5*q3) -
sin(0.5*q2)*sin(0.5*q3)*cos(0.5*q1)
 *
 * Step 4.  Since q2 has already been calculated (in Step 2), subsitute
 *          cos(0.5*q2) = A and sin(0.5*q2) = f*A.
 *          Note: The final results are independent of A.
 *
 * Step 5.  Referring to Step 3 form: (1+f)*e1 + (1+f)*e3 and rearrange to:
 *          sin(0.5*q1+0.5*q3) = (e1+e3)/(A*(1-f))
 *
 *          Referring to Step 3 form: (1+f)*e0 - (1+f)*e2 and rearrange to:
 *          cos(0.5*q1+0.5*q3) = (e0-e2)/(A*(1-f))
 *
 *          Combine the two previous results to produce:
 *          1/2*( q1 + q3 ) = atan2( e1+e3, e0-e2 )
 *
 * Step 6.  Referring to Step 3 form: (1-f)*e1 - (1-f)*e3 and rearrange to:
 *          sin(0.5*q1-0.5*q3) = -(e1-e3)/(A*(1+f))
 *
 *          Referring to Step 3 form: (1-f)*e0 + (1-f)*e2 and rearrange to:
 *          cos(0.5*q1-0.5*q3) = (e0+e2)/(A*(1+f))
 *
 *          Combine the two previous results to produce:
 *          1/2*( q1 - q3 ) = atan2( e3-e1, e0+e2 )
 *
 * Step 7.  Combine Steps 5 and 6 and solve the linear equations for q1, q3.
 *          1/2*( q1 + q3 ) = atan2( e1+e3,  e0-e2 ) = zA
 *          1/2*( q1 - q3 ) = atan2( e3-e1,  e0+e2 ) = zB
 *          q1 = zA + zB
 *          q3 = zA - zB
 *
 * Step 8.  Return angles in the following ranges:
 *          -pi   <= q1 <= pi
 *          -pi/2 <= q2 <= pi/2
 *          -pi   <= q3 <= pi
 *
 * Textbook reference: Advanced Dynamics and Motion Simulation,
 *                     For professional engineers and scientists (2017).
@author Paul Mitiguy
**/
template <typename Derived>
Vector3<typename Derived::Scalar> QuaternionToSpaceXYZ(
    const Eigen::MatrixBase<Derived>& quaternion) {
  // TODO(hongkai.dai@tri.global): Switch to Eigen's Quaternion when we fix
  // the range problem in Eigen
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Eigen::MatrixBase<Derived>, 4);
  Eigen::Matrix3d R = quat2rotmat(quaternion);

  using Scalar = typename Derived::Scalar;
  using std::atan2;
  using std::sqrt;
  using std::abs;

  // This algorithm is specific to BodyZYX order (equivalent to SpaceXYZ),
  // e.g., i=2, j=1, k=0; the given formulas (below) for xA, yA,  xB, yB;
  // and the returned quantities are specific to SpaceXYZ Euler sequence.
  const int i = 2, j = 1, k = 0;

  // Calculate theta2 using lots of information in the rotation matrix.
  // Rsum = abs( cos(theta2) ) is inherently non-negative.
  // Rik = sin(theta2) may be negative, zero, or positive.
  const Scalar Rii = R(i, i);
  const Scalar Rij = R(i, j);
  const Scalar Rjk = R(j, k);
  const Scalar Rkk = R(k, k);
  const Scalar Rsum = sqrt((Rii * Rii + Rij * Rij + Rjk * Rjk + Rkk * Rkk) / 2);
  const Scalar Rik = R(i, k);
  const Scalar theta2 = atan2(-Rik, Rsum);

  // Calculate theta1 and theta3 from Steps 3-7 (documented above).
  const Scalar e0 = quaternion(0), e1 = quaternion(1);
  const Scalar e2 = quaternion(2), e3 = quaternion(3);
  const Scalar yA = e1 + e3, xA = e0 - e2;
  const Scalar yB = e3 - e1, xB = e0 + e2;
  const Scalar epsilon = Eigen::NumTraits<Scalar>::epsilon();
  const bool isSingularA = abs(yA) <= epsilon && abs(xA) <= epsilon;
  const bool isSingularB = abs(yB) <= epsilon && abs(xB) <= epsilon;
  const Scalar zA = isSingularA ? 0.0 : atan2(yA, xA);
  const Scalar zB = isSingularB ? 0.0 : atan2(yB, xB);
  Scalar theta1 = zA + zB;  // First angle in rotation sequence.
  Scalar theta3 = zA - zB;  // Third angle in rotation sequence.

  // If necessary, modify angles theta1 and/or theta3 to be between -pi and pi.
  if (theta1 > M_PI) theta1 = theta1 - 2 * M_PI;
  if (theta1 < -M_PI) theta1 = theta1 + 2 * M_PI;
  if (theta3 > M_PI) theta3 = theta3 - 2 * M_PI;
  if (theta3 < -M_PI) theta3 = theta3 + 2 * M_PI;

  // Return in Drake/ROS conventional SpaceXYZ (roll-pitch-yaw) order
  // (which is equivalent to BodyZYX theta1, theta2, theta3 order).
  Vector3<Scalar> spaceXYZ_angles(theta3, theta2, theta1);

#ifdef DRAKE_ASSERT_IS_ARMED
  // This algorithm converts from quaternion to SpaceXYZ.
  // Test this algorithm by converting the quaternion to a rotation matrix
  // and converting the SpaceXYZ angles to a rotation matrix and ensuring
  // these rotation matrices are within epsilon of each other.
  // Assuming sine, cosine are accurate to 4*(standard double-precision epsilon
  // = 2.22E-16) and there are two sets of two multiplies and one addition for
  // each rotation matrix element, I decided to test with 4.23E-14 epsilon.
  // Note: (1+eps)*(1+eps)*(1+eps) = 1 + 3*eps + 3*eps^2 + eps^3 near 1 + 3*eps,
  // so (1+4*eps)*(1+4*eps)*(1+4*eps) is near 1 + 3*(4^3)*eps = 1 + 192*eps.
  const Matrix3<Scalar> rotMatrix_quaternion = quat2rotmat(quaternion);
  const Matrix3<Scalar> rotMatrix_spaceXYZ = rpy2rotmat(spaceXYZ_angles);
  DRAKE_ASSERT(rotMatrix_quaternion.isApprox(rotMatrix_spaceXYZ, 4.23E-14));
#endif

  return spaceXYZ_angles;
}

/** (Deprecated) Computes SpaceXYZ Euler angles from quaternion.
Use `QuaternionToSpaceXYZ()` instead.
@see QuaternionToSpaceXYZ() **/
template <typename Derived>
Vector3<typename Derived::Scalar> quat2rpy(
    const Eigen::MatrixBase<Derived>& quaternion) {
  return QuaternionToSpaceXYZ(quaternion);
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
