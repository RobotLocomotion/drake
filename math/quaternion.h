/// @file
/// Utilities for arithmetic on quaternions.

#pragma once

#include <cmath>

#include <Eigen/Dense>

#include "drake/common/drake_assert.h"
#include "drake/common/eigen_types.h"
#include "drake/common/is_approx_equal_abstol.h"
#include "drake/math/roll_pitch_yaw_not_using_quaternion.h"
#include "drake/math/rotation_matrix.h"

namespace drake {
namespace math {

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
 * Computes angle-axis orientation from a given quaternion.
 * @tparam Scalar The element type which must be a valid Eigen scalar.
 * @param quaternion 4 x 1 non-zero vector that does not have to be normalized.
 * @return Angle-axis representation of quaternion with 0 <= angle <= PI.
 * and axis as a unit vector. Return is independent of quaternion normalization.
 */
template <typename Scalar>
Eigen::AngleAxis<Scalar> QuaternionToAngleAxis(
    const Eigen::Quaternion<Scalar>& quaternion) {
  // Use Eigen's built-in algorithm which seems robust (checked by Mitiguy/Dai).
  Eigen::AngleAxis<Scalar> angle_axis(quaternion);

  // Before October 2016, Eigen calculated  0 <= angle <= 2*PI.
  // After  October 2016, Eigen calculates  0 <= angle <= PI.
  // Ensure consistency between pre/post October 2016 Eigen versions.
  Scalar& angle = angle_axis.angle();
  Vector3<Scalar>& axis = angle_axis.axis();
  if (angle >= M_PI) {
    angle = 2 * M_PI - angle;
    axis = -axis;
  }

#ifdef DRAKE_ASSERT_IS_ARMED
  // Ensure angle returned is between 0 and PI.
  // const Scalar angle = angle_axis.angle();
  DRAKE_ASSERT(0.0 <= angle && angle <= M_PI);

  // Ensure a unit vector is returned, i.e., magnitude 1.
  // const Vector3<Scalar> axis = angle_axis.axis();
  const Scalar norm = axis.norm();
  // Normalization of Vector3 has 3 multiplies, 2 additions and one sqrt.
  // Each multiply has form (1+eps)*(1+eps) = 1 + 2*eps + eps^2.
  // Each + or * or sqrt rounds-off, which can introduce 1/2 eps for each.
  // Use: (3 mult * 2*eps) + (3 mults + 2 adds + 1 sqrt) * 1/2 eps = 9 eps.
  const Scalar epsilon = Eigen::NumTraits<Scalar>::epsilon();
  using std::abs;
  DRAKE_ASSERT(abs(norm - 1) < 9 * epsilon);
#endif

  return angle_axis;
}

/**
 * (Deprecated) Computes axis-angle orientation from a given quaternion.
 * @tparam Derived An Eigen derived type, e.g., an Eigen Vector3d.
 * @param quaternion 4 x 1 vector that may or may not be normalized.
 * @return axis-angle [x; y; z; angle] of quaternion with axis as a unit vector
 * and  0 <= angle <= PI,  Return is independent of quaternion normalization.
 * (Deprecated) Use `QuaternionToAngleAxis()` instead.
 * @see QuaternionToAngleAxis()
 */
template <typename Derived>
Vector4<typename Derived::Scalar> quat2axis(
    const Eigen::MatrixBase<Derived>& quaternion) {
  // TODO(hongkai.dai@tri.global): Switch to Eigen's Quaternion when we fix
  // the range problem in Eigen
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Eigen::MatrixBase<Derived>, 4);

  using Scalar = typename Derived::Scalar;
  Eigen::Quaternion<Scalar> eigen_quaternion(quaternion(0), quaternion(1),
                                             quaternion(2), quaternion(3));

  // Switch Eigen angleAxis [angle,x,y,z] order to Drake axisAngle
  // [x,y,z,angle].
  const Eigen::AngleAxis<Scalar> aa = QuaternionToAngleAxis(eigen_quaternion);
  Vector4<Scalar> axis_angle;
  axis_angle(3) = aa.angle();  // Drake's last element is angle.
  axis_angle.template head<3>() =
      aa.axis();  // Drake's first elements are axis.

  return axis_angle;
}

/**
 * Computes the rotation matrix from quaternion representation.
 * @tparam Derived An Eigen derived type, e.g., an Eigen Vector3d.
 * @param quaternion 4 x 1 unit length quaternion, @p q=[w;x;y;z]
 * @return 3 x 3 rotation matrix
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
 * @tparam Derived An Eigen derived type, e.g., an Eigen Vector3d.
 * @param quaternion 4x1 unit length vector with elements [ e0, e1, e2, e3 ].
 * @return 3x1 SpaceXYZ Euler angles (called roll-pitch-yaw by ROS).
 *
 * This accurate algorithm avoids numerical round-off issues encountered by
 * some algorithms when pitch angle is within 1E-6 of PI/2 or -PI/2.
 *
 * Note: SpaceXYZ roll-pitch-yaw is equivalent to BodyZYX yaw-pitch-roll.
 * http://answers.ros.org/question/58863/incorrect-rollpitch-yaw-values-using-getrpy/
 *
 * <h3>Theory</h3>
 *
 * This algorithm was created October 2016 by Paul Mitiguy for TRI (Toyota).
 * We believe this is a new algorithm (not previously published).
 * Some of the theory/formulation of this algorithm are provided below.
 *
 * <pre>
 * Notation: Angles q1, q2, q3 designate SpaceXYZ "roll, pitch, yaw" angles.
 *           Symbols e0, e1, e2, e3 are elements of the passed-in quaternion.
 *           e0 = cos(theta/2), e1 = L1*sin(theta/2), e2 = L2*sin(theta/2), ...
 *
 * Step 1.  Convert the quaternion to a 3x3 rotation matrix R.
 *          This is done solely to provide an accurate computation of pitch-
 *          angle q2, which is calculated with the atan2 function and only 5
 *          elements of what is interpretated as a SpaceXYZ rotation matrix.
 *          Since only 5 elements of R are used, perhaps the algorithm could
 *          be improved by only calculating those 5 elements -- or manipulating
 *          those 5 elements to reduce calculations involving e0, e1, e2, e3.
 *
 * Step 2.  Realize the quaternion passed to the function can be regarded as
 *          resulting from multiplication of certain 4x4 and 4x1 matrices, or
 *          multiplying three rotation quaternions (Hamilton product), to give:
 *          e0 = sin(q1/2)*sin(q2/2)*sin(q3/2) + cos(q1/2)*cos(q2/2)*cos(q3/2)
 *          e1 = sin(q3/2)*cos(q1/2)*cos(q2/2) - sin(q1/2)*sin(q2/2)*cos(q3/2)
 *          e2 = sin(q1/2)*sin(q3/2)*cos(q2/2) + sin(q2/2)*cos(q1/2)*cos(q3/2)
 *          e3 = sin(q1/2)*cos(q2/2)*cos(q3/2) - sin(q2/2)*sin(q3/2)*cos(q1/2)
 *
 *          Reference for step 2:
 * https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
 *
 * Step 3.  Since q2 has already been calculated (in Step 1), substitute
 *          cos(q2/2) = A and sin(q2/2) = f*A.
 *          Note: The final results are independent of A and f = tan(q2/2).
 *          Note: -pi/2 <= q2 <= pi/2  so -0.707 <= [A = cos(q2/2)] <= 0.707...
 *          and  -1 <= [f = tan(q2/2)] <= 1.
 *
 * Step 4.  Referring to Step 2 form: (1+f)*e1 + (1+f)*e3 and rearrange to:
 *          sin(q1/2+q3/2) = (e1+e3)/(A*(1-f))
 *
 *          Referring to Step 2 form: (1+f)*e0 - (1+f)*e2 and rearrange to:
 *          cos(q1/2+q3/2) = (e0-e2)/(A*(1-f))
 *
 *          Combine the two previous results to produce:
 *          1/2*( q1 + q3 ) = atan2( e1+e3, e0-e2 )
 *
 * Step 5.  Referring to Step 2 form: (1-f)*e1 - (1-f)*e3 and rearrange to:
 *          sin(q1/5-q3/5) = -(e1-e3)/(A*(1+f))
 *
 *          Referring to Step 2 form: (1-f)*e0 + (1-f)*e2 and rearrange to:
 *          cos(q1/2-q3/2) = (e0+e2)/(A*(1+f))
 *
 *          Combine the two previous results to produce:
 *          1/2*( q1 - q3 ) = atan2( e3-e1, e0+e2 )
 *
 * Step 6.  Combine Steps 4 and 5 and solve the linear equations for q1, q3.
 *          Use zA, zB to handle case in which both atan2 arguments are 0.
 *          zA = (e1+e3==0  &&  e0-e2==0) ? 0 : atan2( e1+e3, e0-e2 );
 *          zB = (e3-e1==0  &&  e0+e2==0) ? 0 : atan2( e3-e1, e0+e2 );
 *          Solve: 1/2*( q1 + q3 ) = zA     To produce:  q1 = zA + zB
 *                 1/2*( q1 - q3 ) = zB                  q3 = zA - zB
 *
 * Step 7.  As necessary, modify angles by 2*PI to return angles in range:
 *          -pi   <= q1 <= pi
 *          -pi/2 <= q2 <= pi/2
 *          -pi   <= q3 <= pi
 *
 * Textbook reference: Mitiguy, Paul, Advanced Dynamics and Motion Simulation,
 *                     For professional engineers and scientists (2017).
 *                     Section 8.2, Euler rotation angles, pg 60.
 *                     Available at www.MotionGenesis.com
 * </pre>
 * @author Paul Mitiguy
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

  // This algorithm is specific to SpaceXYZ order, including the calculation
  // of q2, the formulas for xA,yA, xB,yB, and values of q1, q3.
  // It is easily modified for other SpaceIJK and BodyIJI rotation sequences.

  // Calculate q2 using lots of information in the rotation matrix.
  // Rsum = abs( cos(q2) ) is inherently non-negative.
  // R20 = -sin(q2) may be negative, zero, or positive.
  const Scalar R22 = R(2, 2);
  const Scalar R21 = R(2, 1);
  const Scalar R10 = R(1, 0);
  const Scalar R00 = R(0, 0);
  const Scalar Rsum = sqrt((R22 * R22 + R21 * R21 + R10 * R10 + R00 * R00) / 2);
  const Scalar R20 = R(2, 0);
  const Scalar q2 = atan2(-R20, Rsum);

  // Calculate q1 and q3 from Steps 2-6 (documented above).
  const Scalar e0 = quaternion(0), e1 = quaternion(1);
  const Scalar e2 = quaternion(2), e3 = quaternion(3);
  const Scalar yA = e1 + e3, xA = e0 - e2;
  const Scalar yB = e3 - e1, xB = e0 + e2;
  const Scalar epsilon = Eigen::NumTraits<Scalar>::epsilon();
  const bool isSingularA = abs(yA) <= epsilon && abs(xA) <= epsilon;
  const bool isSingularB = abs(yB) <= epsilon && abs(xB) <= epsilon;
  const Scalar zA = isSingularA ? 0.0 : atan2(yA, xA);
  const Scalar zB = isSingularB ? 0.0 : atan2(yB, xB);
  Scalar q1 = zA - zB;  // First angle in rotation sequence.
  Scalar q3 = zA + zB;  // Third angle in rotation sequence.

  // If necessary, modify angles q1 and/or q3 to be between -pi and pi.
  if (q1 > M_PI) q1 = q1 - 2 * M_PI;
  if (q1 < -M_PI) q1 = q1 + 2 * M_PI;
  if (q3 > M_PI) q3 = q3 - 2 * M_PI;
  if (q3 < -M_PI) q3 = q3 + 2 * M_PI;

  // Return in Drake/ROS conventional SpaceXYZ q1, q2, q3 (roll-pitch-yaw) order
  // (which is equivalent to BodyZYX q3, q2, q1 order).
  Vector3<Scalar> spaceXYZ_angles(q1, q2, q3);

#ifdef DRAKE_ASSERT_IS_ARMED
  // This algorithm converts from quaternion to SpaceXYZ.
  // Test this algorithm by converting the quaternion to a rotation matrix
  // and converting the SpaceXYZ angles to a rotation matrix and ensuring
  // these rotation matrices are within epsilon of each other.
  // Assuming sine, cosine are accurate to 4*(standard double-precision epsilon
  // = 2.22E-16) and there are two sets of two multiplies and one addition for
  // each rotation matrix element, I decided to test with 20 * epsilon:
  // (1+4*eps)*(1+4*eps)*(1+4*eps) = 1 + 3*(4*eps) + 3*(4*eps)^2 + (4*eps)^3.
  // Each + or * or sqrt rounds-off, which can introduce 1/2 eps for each.
  // Use: (12*eps) + (4 mults + 1 add) * 1/2 eps = 17.5 eps.
  const Matrix3<Scalar> rotMatrix_quaternion = quat2rotmat(quaternion);
  const Matrix3<Scalar> rotMatrix_spaceXYZ = rpy2rotmat(spaceXYZ_angles);
  DRAKE_ASSERT(rotMatrix_quaternion.isApprox(rotMatrix_spaceXYZ, 20 * epsilon));
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
