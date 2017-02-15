// Purpose: Compare Drake with analytical closed-form solution from Section
//          1.13 (pgs. 60-62) of Spacecraft Dynamics, by Kane & Levinson.
//          This closed-form solution includes both angular velocity and
//          Euler parameters for an axis-symmetric rigid body B, when the moment
//          of forces on B about Bcm (B's center of mass) is zero (torque-free).
//-----------------------------------------------------------------------------
#include <cmath>
#include <tuple>
#include <gtest/gtest.h>

#include "drake/common/drake_path.h"
#include "drake/common/eigen_matrix_compare.h"
#include "drake/common/eigen_types.h"
#include "drake/math/quaternion.h"
#include "drake/math/autodiff.h"
#include "drake/multibody/joints/floating_base_types.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_plant/rigid_body_plant.h"
#include "drake/systems/analysis/runge_kutta3_integrator.h"
#include "drake/systems/analysis/runge_kutta3_integrator-inl.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/constant_vector_source.h"

namespace drake {
namespace benchmarks {
namespace cylinder_torque_free_analytical_solution {
using Eigen::Vector3d;
using Eigen::Vector4d;
using Eigen::VectorXd;
using Eigen::Quaterniond;

/**
 * This function tests whether a quaternion in its "canonical form" meaning that
 * for a quaternion [e0, e1, e2, e3], its 0th term is inherently non-negative.
 * @param quat Quaternion e0, e1, e2, e3 that relates two right-handed
 *   orthogonal unitary bases e.g., Ax, Ay, Az (A) to Bx, By, Bz (B).
 *   Note: quat is analogous to the rotation matrix R_AB.
 * @return True if quat.w() is positive (in canonical form), otherwise false.
 */
template<typename T>
bool IsQuaternionInCanonicalForm(const Eigen::Quaternion<T>& quat) {
  return quat.w() >= 0.0;
}


// This function tests whether a quaternion is in canonical form, which for a
// quaternion [e0, e1, e2, e3], it tests whether the e0 term is non-negative.
GTEST_TEST(IsQuaternionInCanonicalFormTest, testA) {
  const double half_angle = M_PI/6;
  const double sin_half_angle = std::sin(half_angle);
  const double x = 0.3, y = 0.7, z = std::sqrt(1.0 - (x*x + y*y));
  const double e0 = std::cos(half_angle);
  const double e1 = sin_half_angle * x;
  const double e2 = sin_half_angle * y;
  const double e3 = sin_half_angle * z;
  const Eigen::Quaterniond a(+e0, +e1, +e2, +e3);
  const Eigen::Quaterniond b(-e0, -e1, -e2, -e3);
  EXPECT_TRUE(IsQuaternionInCanonicalForm(a));
  EXPECT_FALSE(IsQuaternionInCanonicalForm(b));
}


/**
 * This function returns a quaternion in its "canonical form" meaning that it
 * returns a quaternion [e0, e1, e2, e3] with a non-negative e0.
 * @param quat Quaternion [e0, e1, e2, e3] that relates two right-handed
 *   orthogonal unitary bases e.g., Ax, Ay, Az (A) to Bx, By, Bz (B).
 *   Note: quat is analogous to the rotation matrix R_AB.
 * @return Canonical form of quat, which means that either the original quat
 *   is returned or a quaternion representing the same orientation but with
 *   negated [e0, e1, e2, e3], to ensure a positive e0 in returned quaternion.
 */
template<typename T>
Eigen::Quaternion<T> QuaternionToCanonicalForm(
                     const Eigen::Quaternion<T>& quat ) {
  return IsQuaternionInCanonicalForm(quat) ? quat :
         Eigen::Quaternion<T>(-quat.w(), -quat.x(), -quat.y(), -quat.z());
}


// This function tests conversion of a quaternion to its canonical form, meaning
// it tests conversion to a quaternion [e0, e1, e2, e3] with non-negative e0.
GTEST_TEST(QuaternionToCanonicalFormTest, testA) {
  const double half_angle = M_PI/6;
  const double sin_half_angle = std::sin(half_angle);
  const double x = 0.3, y = 0.7, z = std::sqrt(1.0 - (x*x + y*y));
  const double e0 = std::cos(half_angle);
  const double e1 = sin_half_angle * x;
  const double e2 = sin_half_angle * y;
  const double e3 = sin_half_angle * z;
  const Eigen::Quaterniond a(+e0, +e1, +e2, +e3);
  const Eigen::Quaterniond b(-e0, -e1, -e2, -e3);
  const Eigen::Quaterniond a_canonical = QuaternionToCanonicalForm(a);
  const Eigen::Quaterniond b_canonical = QuaternionToCanonicalForm(b);
  EXPECT_TRUE(a_canonical.isApprox(a));
  EXPECT_FALSE(b_canonical.isApprox(b));
}

/**
 * This function tests whether the canonical form of one quaternion is equal to
 * the canonical form of another quaternion, which is useful for testing whether
 * two quaternions represent the same orientation.
 * @param quat1 Quaternion e0, e1, e2, e3 that relates two right-handed
 *   orthogonal unitary bases e.g., Ax, Ay, Az (A) to Bx, By, Bz (B).
 *   Note: quat is analogous to the rotation matrix R_AB.
  * @param quat2 Quaternion e0, e1, e2, e3 that relates two right-handed
 *   orthogonal unitary bases e.g., Ax, Ay, Az (A) to Bx, By, Bz (B).
 *   Note: quat is analogous to the rotation matrix R_AB.*
 * @return True if quat.w() is positive (in canonical form), otherwise false.
 */
template<typename T>
bool AreQuaternionsApproximatlyEqualInCanonicalForm(
     const Eigen::Quaternion<T>& quat1,
     const Eigen::Quaternion<T>& quat2,
     const T tolerance) {
  const Eigen::Quaternion<T> quat1_canonical = QuaternionToCanonicalForm(quat1);
  const Eigen::Quaternion<T> quat2_canonical = QuaternionToCanonicalForm(quat2);
  return quat1_canonical.isApprox(quat2_canonical, tolerance);
}


// This function tests whether a quaternion is in canonical form, which for a
// quaternion [e0, e1, e2, e3], it tests whether the e0 term is non-negative.
GTEST_TEST(AreQuaternionsApproximatlyEqualInCanonicalFormTest, testA) {
  const double half_angle = M_PI/6;
  const double sin_half_angle_a = std::sin(half_angle);
  const double x = 0.3, y = 0.7, z = std::sqrt(1.0 - (x*x + y*y));
  const double e0a = std::cos(half_angle);
  const double e1a = sin_half_angle_a * x;
  const double e2a = sin_half_angle_a * y;
  const double e3a = sin_half_angle_a * z;
  const Eigen::Quaterniond a(+e0a, +e1a, +e2a, +e3a);
  const Eigen::Quaterniond b(-e0a, -e1a, -e2a, -e3a);
  const double sin_half_angle_c = std::sin(0.99*half_angle);
  const double e0c = std::cos(0.99*half_angle);
  const double e1c = sin_half_angle_c * x;
  const double e2c = sin_half_angle_c * y;
  const double e3c = sin_half_angle_c * z;
  const Eigen::Quaterniond c(+e0c, +e1c, +e2c, +e3c);
  const double epsilon = std::numeric_limits<double>::epsilon();
  EXPECT_TRUE(AreQuaternionsApproximatlyEqualInCanonicalForm(a, b, 8*epsilon));
  EXPECT_FALSE(AreQuaternionsApproximatlyEqualInCanonicalForm(a, c, 0.001));
}

/** This function calculates a quaternion's time-derivative from its quaternion
 * and angular velocity. Algorithm from [Kane, 1983] Section 1.13, Pages 58-59.
 * @param quat_AB Quaternion e0, e1, e2, e3 that relates two right-handed
 *   orthogonal unitary bases e.g., Ax, Ay, Az (A) to Bx, By, Bz (B).
 *   Note: quat_AB is analogous to the rotation matrix R_AB.
 * @param w_AB_B  B's angular velocity in A, expressed in B.
 * @retval quatDt time-derivative of quat_AB, i.e., [e0', e1', e2', e3'].
 *
 * @note To avoid dependence on Eigen's internal ordering of elements in its
 * Quaternion class, herein we use `e0 = quat.w()', `e1 = quat.x()`, etc.
 * Return value `quatDt` *does* have a specific order as defined above.
 *
 * - [Kane, 1983] "Spacecraft Dynamics," McGraw-Hill Book Co., New York, 1983.
 *   (With P. W. Likins and D. A. Levinson).  Available for free .pdf download:
 *   https://ecommons.cornell.edu/handle/1813/637
 */
// TODO(mitiguy) Move this and related methods (make unit test) to quaternion.h.
// TODO(mitiguy and Dai)  Create QuaternionDt class and update Doxygen.
template<typename T>
Vector4<T> CalculateQuaternionDtFromAngularVelocityExpressedInB(
           const Eigen::Quaternion<T>& quat_AB,  const Vector3<T>& w_AB_B ) {
  const T e0 = quat_AB.w(),  e1 = quat_AB.x(),
          e2 = quat_AB.y(),  e3 = quat_AB.z();
  const T wx = w_AB_B[0], wy = w_AB_B[1], wz = w_AB_B[2];

  const T e0Dt = 0.5*(-e1*wx - e2*wy - e3*wz);
  const T e1Dt = 0.5 *(e0*wx - e3*wy + e2*wz);
  const T e2Dt = 0.5 *(e3*wx + e0*wy - e1*wz);
  const T e3Dt = 0.5*(-e2*wx + e1*wy + e0*wz);

  return Vector4<T>(e0Dt, e1Dt, e2Dt, e3Dt);
}


// This function tests CalculateQuaternionDtFromAngularVelocityExpressedInB.
GTEST_TEST(CalculateQuaternionDtFromAngularVelocityExpressedInBTest, testA) {
  const double half_angle = M_PI/6;
  const double sin_half_angle = std::sin(half_angle);
  const double x = 0.3, y = 0.7, z = std::sqrt(1.0 - (x*x + y*y));

  // For convenience, locally typedef Vector7d and AutoDiff7d.
  using Vector7d = Eigen::Matrix<double, 7, 1>;
  using AutoDiff7d = Eigen::AutoDiffScalar<Vector7d>;

  // Initialize each variable that is to be regarded as an independent variable
  // (for purposes of partial differentiation) with its numerical value (i.e.,
  // (where the function and its nth derivatives are to be evaluated) and the
  // appropriate entry for it in the 7d array.
  const AutoDiff7d e0(std::cos(half_angle), 7, 0);
  const AutoDiff7d e1(sin_half_angle * x,   7, 1);
  const AutoDiff7d e2(sin_half_angle * y,   7, 2);
  const AutoDiff7d e3(sin_half_angle * z,   7, 3);
  const AutoDiff7d wx(2.1,                  7, 4);
  const AutoDiff7d wy(-3.4,                 7, 5);
  const AutoDiff7d wz(5.3,                  7, 6);
  const Eigen::Quaternion<AutoDiff7d> quat(e0,  e1,  e2,  e3);
  const drake::Vector3<AutoDiff7d> w_B(wx, wy, wz);
  const drake::Vector4<AutoDiff7d> quatDt =
      CalculateQuaternionDtFromAngularVelocityExpressedInB(quat, w_B);

  // Verify Drake's quatDt versus MotionGenesis (MG) results.
  const Eigen::Vector4d quatDt_MG(-0.4211981425390414,  2.387689633338328,
                                  -1.529504299767133,   1.672467320028763);
  const Eigen::Vector4d quatDt_numerical = math::autoDiffToValueMatrix(quatDt);
  const double epsilon = std::numeric_limits<double>::epsilon();
  EXPECT_TRUE(CompareMatrices(quatDt_numerical, quatDt_MG, 8*epsilon));

  // Test Drake partials of quatDt with respect to [e0, e1, e2, e3, wx, wy, wz].
  const Vector7d e0Dt_partials = quatDt(0).derivatives();
  const Vector7d e1Dt_partials = quatDt(1).derivatives();
  const Vector7d e2Dt_partials = quatDt(2).derivatives();
  const Vector7d e3Dt_partials = quatDt(3).derivatives();

  // MotionGenesis was used to calculate and evaluate same partial derivatives.
  Vector7d e0Dt_partial_MG, e1Dt_partial_MG, e2Dt_partial_MG, e3Dt_partial_MG;
  e0Dt_partial_MG.head<4>() <<  0,    -1.05,  1.7,  -2.65;
  e1Dt_partial_MG.head<4>() <<  1.05,  0,     2.65,  1.7;
  e2Dt_partial_MG.head<4>() << -1.7,  -2.65,  0,     1.05;
  e3Dt_partial_MG.head<4>() <<  2.65, -1.7,  -1.05,  0;
  e0Dt_partial_MG.tail<3>() << -0.075,              -0.175,
                               -0.1620185174601965;
  e1Dt_partial_MG.tail<3>() <<  0.4330127018922194, -0.1620185174601965,  0.175;
  e2Dt_partial_MG.tail<3>() <<  0.1620185174601965,  0.4330127018922194, -0.075;
  e3Dt_partial_MG.tail<3>() << -0.175,               0.075,
                                0.4330127018922194;

  // Verify Drake's partials are nearly identical to exact values.
  EXPECT_TRUE(CompareMatrices(e0Dt_partials, e0Dt_partial_MG, 8*epsilon));
  EXPECT_TRUE(CompareMatrices(e1Dt_partials, e1Dt_partial_MG, 8*epsilon));
  EXPECT_TRUE(CompareMatrices(e2Dt_partials, e2Dt_partial_MG, 8*epsilon));
  EXPECT_TRUE(CompareMatrices(e3Dt_partials, e3Dt_partial_MG, 8*epsilon));
}


/** This function calculates angular velocity from a quaternion and its time-
 * derivative. Algorithm from [Kane, 1983] Section 1.13, Pages 58-59.
 * @param quat_AB  Quaternion e0, e1, e2, e3 that relates two right-handed
 *   orthogonal unitary bases e.g., Ax, Ay, Az (A) to Bx, By, Bz (B).
 *   Note: quat_AB is analogous to the rotation matrix R_AB.
 * @param quatDt  time-derivative of `quat_AB`, i.e. [e0', e1', e2', e3'].
 * @retval w_AB_B  B's angular velocity in A, expressed in B.
 *
 * @note To avoid dependence on Eigen's internal ordering of elements in its
 * Quaternion class, herein we use `e0 = quat.w()', `e1 = quat.x()`, etc.
 * Parameter `quatDt` *does* have a specific order as defined above.
 *
 * - [Kane, 1983] "Spacecraft Dynamics," McGraw-Hill Book Co., New York, 1983.
 *   (with P. W. Likins and D. A. Levinson).  Available for free .pdf download:
 *   https://ecommons.cornell.edu/handle/1813/637
 */
// TODO(mitiguy) Move this and related methods (make unit test) to quaternion.h.
// TODO(mitiguy and Dai)  Create QuaternionDt class and update Doxygen.
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


// This function tests CalculateQuaternionDtFromAngularVelocityExpressedInB.
GTEST_TEST(CalculateAngularVelocityExpressedInBFromQuaternionDtTest, testA) {
  const double half_angle = M_PI/6;
  const double sin_half_angle = std::sin(half_angle);
  const double x = 0.3, y = 0.7, z = std::sqrt(1.0 - (x*x + y*y));

  // For convenience, locally typedef Vector7d and AutoDiff7d.
  using Vector8d = Eigen::Matrix<double, 8, 1>;
  using AutoDiff8d = Eigen::AutoDiffScalar<Vector8d>;

  // Initialize each variable that is to be regarded as an independent variable
  // (for purposes of partial differentiation) with its numerical value (i.e.,
  // (where the function and its nth derivatives are to be evaluated) and the
  // appropriate entry for it in the 7d array.
  const AutoDiff8d e0(std::cos(half_angle),  8, 0);
  const AutoDiff8d e1(sin_half_angle * x,    8, 1);
  const AutoDiff8d e2(sin_half_angle * y,    8, 2);
  const AutoDiff8d e3(sin_half_angle * z,    8, 3);
  const AutoDiff8d e0Dt(-0.4211981425390414, 8, 4);
  const AutoDiff8d e1Dt(+2.387689633338328,  8, 5);
  const AutoDiff8d e2Dt(-1.529504299767133,  8, 6);
  const AutoDiff8d e3Dt(+1.672467320028763,  8, 7);
  const Eigen::Quaternion<AutoDiff8d> quat(e0,  e1,  e2,  e3);
  const drake::Vector4<AutoDiff8d> quatDt(e0Dt, e1Dt, e2Dt, e3Dt);
  const drake::Vector3<AutoDiff8d> w_B =
      CalculateAngularVelocityExpressedInBFromQuaternionDt(quat, quatDt);

  // Verify Drake's w_B versus exact results.
  const Eigen::Vector3d w_B_exact(2.1, -3.4, 5.3);
  const Eigen::Vector3d w_B_numerical = math::autoDiffToValueMatrix(w_B);
  const double epsilon = std::numeric_limits<double>::epsilon();
  EXPECT_TRUE(CompareMatrices(w_B_numerical, w_B_exact, 8*epsilon));
}


/** This function calculates how well a quaternion and its time-derivative
 * satisfy the quaternion time-derivative constraint specified in [Kane, 1983]
 * Section 1.13, equations 12-13, page 59.  For a quaternion [e0, e1, e2, e3],
 * the quaternion must satisfy:  e0^2 + e1^2 + e2^2 + e3^2 = 1,   hence its
 * time-derivative must satisfy:  2*(e0*e0' + e1*e1' + e2*e2' + e3*e3') = 0.
 * @param quat  Quaternion e0, e1, e2, e3 that relates two right-handed
 *   orthogonal unitary bases e.g., Ax, Ay, Az (A) to Bx, By, Bz (B).
 *   Note: A quaternion like quat_AB is analogous to the rotation matrix R_AB.
 * @param quatDt  time-derivative of `quat`, i.e., [e0', e1', e2', e3'].
 * @retval value of constraint - should be near 0 (may be positive or negative).
 *
 * - [Kane, 1983] "Spacecraft Dynamics," McGraw-Hill Book Co., New York, 1983.
 *   (with P. W. Likins and D. A. Levinson).  Available for free .pdf download:
 *   https://ecommons.cornell.edu/handle/1813/637
 */
// TODO(mitiguy) Move this and related methods (make unit test) to quaternion.h.
template <typename T>
T CalculateQuaternionDtConstraintViolation(const Eigen::Quaternion<T>& quat,
                                           const Vector4<T>& quatDt) {
  const T e0 = quat.w(), e1 = quat.x(), e2 = quat.y(), e3 = quat.z();
  const T e0Dt = quatDt[0], e1Dt = quatDt[1],
          e2Dt = quatDt[2], e3Dt = quatDt[3];
  return 2.0 * (e0*e0Dt + e1*e1Dt + e2*e2Dt + e3*e3Dt);
}


// This function tests CalculateQuaternionDtConstraintViolation.
GTEST_TEST(CalculateQuaternionDtConstraintViolationTest, testA) {
  const double half_angle = M_PI/6;
  const double sin_half_angle = std::sin(half_angle);
  const double x = 0.3, y = 0.7, z = std::sqrt(1.0 - (x*x + y*y));

  // For convenience, locally typedef Vector7d and AutoDiff7d.
  using Vector8d = Eigen::Matrix<double, 8, 1>;
  using AutoDiff8d = Eigen::AutoDiffScalar<Vector8d>;

  // Initialize each variable that is to be regarded as an independent variable
  // (for purposes of partial differentiation) with its numerical value (i.e.,
  // (where the function and its nth derivatives are to be evaluated) and the
  // appropriate entry for it in the 7d array.
  const AutoDiff8d e0(std::cos(half_angle),  8, 0);
  const AutoDiff8d e1(sin_half_angle * x,    8, 1);
  const AutoDiff8d e2(sin_half_angle * y,    8, 2);
  const AutoDiff8d e3(sin_half_angle * z,    8, 3);
  const AutoDiff8d e0Dt(-0.4211981425390414, 8, 4);
  const AutoDiff8d e1Dt(+2.387689633338328,  8, 5);
  const AutoDiff8d e2Dt(-1.529504299767133,  8, 6);
  const AutoDiff8d e3Dt(+1.672467320028763,  8, 7);
  const Eigen::Quaternion<AutoDiff8d> quat(e0,  e1,  e2,  e3);
  const drake::Vector4<AutoDiff8d> quatDt(e0Dt, e1Dt, e2Dt, e3Dt);
  const drake::Vector4<AutoDiff8d> quatDt_b(quatDt[0], quatDt[1], quatDt[2],
                                       0.99*quatDt[3]);
  const AutoDiff8d a = CalculateQuaternionDtConstraintViolation(quat, quatDt);
  const AutoDiff8d b = CalculateQuaternionDtConstraintViolation(quat, quatDt_b);
  const double a_value = a.value();
  const double b_value = b.value();

  // For the given numbers, a_value should be near zero, whereas b_value is not.
  const double epsilon = std::numeric_limits<double>::epsilon();
  EXPECT_TRUE(std::abs(a_value) <=  8*epsilon);
  EXPECT_TRUE(std::abs(b_value) >=  0.01);
}


/** This function tests if a quaternion and its time-derivative satisfy the
 * quaternion and its time-derivative constraint specified in [Kane, 1983]
 * Section 1.13, equations 12-13, page 59.  For a quaternion [e0, e1, e2, e3],
 * the quaternion must satisfy:  e0^2 + e1^2 + e2^2 + e3^2 = 1,   hence its
 * time-derivative must satisfy:  2*(e0*e0' + e1*e1' + e2*e2' + e3*e3') = 0.
 * @param quat  Quaternion e0, e1, e2, e3 that relates two right-handed
 *   orthogonal unitary bases e.g., Ax, Ay, Az (A) to Bx, By, Bz (B).
 *   Note: A quaternion like quat_AB is analogous to the rotation matrix R_AB.
 * @param quatDt  time-derivative of `quat`, i.e., [e0', e1', e2', e3'].
 * @param tolerance Tolerance required to match results.
 * @returns true if both of the following constraints are satisfied:
 * a) e0^2 + e1^2 + e3^2 + e3^2 - 1 = 0,  to within tolerance.
 * b) 2*(e0*e0' + e1*e1' + e2*e2' + e3*e3') = 0   to within tolerance.
 *
 * - [Kane, 1983] "Spacecraft Dynamics," McGraw-Hill Book Co., New York, 1983.
 *   (with P. W. Likins and D. A. Levinson).  Available for free .pdf download:
 *   https://ecommons.cornell.edu/handle/1813/637
 */
// TODO(mitiguy) Move this and related methods (make unit test) to quaternion.h.
template <typename T>
bool IsBothQuaternionAndQuaternionDtOK(const Eigen::Quaternion<T>& quat,
                                       const Vector4<T>& quatDt,
                                       const double tolerance) {
  using std::abs;

  // For an accurate test, the quaternion should be reasonably accurate.
  const T quat_norm_error = abs(1.0 - quat.norm());
  const bool is_good_quat_norm = (quat_norm_error <= tolerance);
  if ( is_good_quat_norm == false ) return false;

  const T quatDt_test =
               CalculateQuaternionDtConstraintViolation(quat, quatDt);
  return abs(quatDt_test) <= tolerance;
}


// This function tests CalculateQuaternionDtConstraintViolation.
GTEST_TEST(IsBothQuaternionAndQuaternionDtOK, testA) {
  const double half_angle = M_PI/6;
  const double sin_half_angle = std::sin(half_angle);
  const double x = 0.3, y = 0.7, z = std::sqrt(1.0 - (x*x + y*y));

  // For convenience, locally typedef Vector7d and AutoDiff7d.
  using Vector8d = Eigen::Matrix<double, 8, 1>;
  using AutoDiff8d = Eigen::AutoDiffScalar<Vector8d>;

  // Initialize each variable that is to be regarded as an independent variable
  // (for purposes of partial differentiation) with its numerical value (i.e.,
  // (where the function and its nth derivatives are to be evaluated) and the
  // appropriate entry for it in the 7d array.
  const AutoDiff8d e0(std::cos(half_angle),  8, 0);
  const AutoDiff8d e1(sin_half_angle * x,    8, 1);
  const AutoDiff8d e2(sin_half_angle * y,    8, 2);
  const AutoDiff8d e3(sin_half_angle * z,    8, 3);
  const AutoDiff8d e0Dt(-0.4211981425390414, 8, 4);
  const AutoDiff8d e1Dt(+2.387689633338328,  8, 5);
  const AutoDiff8d e2Dt(-1.529504299767133,  8, 6);
  const AutoDiff8d e3Dt(+1.672467320028763,  8, 7);
  const Eigen::Quaternion<AutoDiff8d> quat(e0,  e1,  e2,  e3);
  const drake::Vector4<AutoDiff8d> quatDt(e0Dt, e1Dt, e2Dt, e3Dt);
  const drake::Vector4<AutoDiff8d> quatDt_b(quatDt[0], quatDt[1], quatDt[2],
                                       0.99*quatDt[3]);

  const double epsilon = std::numeric_limits<double>::epsilon();
  const bool a = IsBothQuaternionAndQuaternionDtOK(quat, quatDt,   8*epsilon);
  const bool b = IsBothQuaternionAndQuaternionDtOK(quat, quatDt_b, 0.01);
  EXPECT_TRUE(a);
  EXPECT_FALSE(b);
}


//-----------------------------------------------------------------------------
// TODO(Mitiguy) Remove function unless useful for more than Evan's PR# 4604.
// If more generally useful, create Doxygen for this function.
//-----------------------------------------------------------------------------
void  TestMapVelocityToQDot(
    const drake::systems::RigidBodyPlant<double>& rigid_body_plant,
    const drake::systems::Context<double>& context,
    const Vector3d& w_NB_B_exact,
    const Vector3d& v_NBo_B_exact,
    const Vector4d& quatDt_NB_exact,
    const Vector3d& xyzDt_exact,
    const double tolerance) {

  // Form matrix of motion variables.
  Eigen::VectorXd motion_variables(6);
  motion_variables << w_NB_B_exact, v_NBo_B_exact;

  // Test whether MapVelocityToQDot accurately converts motion variables to
  // time-derivatives of coordinates.
  systems::BasicVector<double> coordinatesDt_from_map(7);
  rigid_body_plant.MapVelocityToQDot(context, motion_variables,
                                     &coordinatesDt_from_map);
  const double xDt_map = coordinatesDt_from_map[0];
  const double yDt_map = coordinatesDt_from_map[1];
  const double zDt_map = coordinatesDt_from_map[2];
  const double e0Dt_map = coordinatesDt_from_map[3];
  const double e1Dt_map = coordinatesDt_from_map[4];
  const double e2Dt_map = coordinatesDt_from_map[5];
  const double e3Dt_map = coordinatesDt_from_map[6];
  const Vector3d xyzDt_map(xDt_map, yDt_map, zDt_map);
  const Vector4d quatDt_map(e0Dt_map, e1Dt_map, e2Dt_map, e3Dt_map);
#if 0  // TODO(mitiguy) Remove these debug statements.
  std::cout << "\n\n--------------------------------------------------";
  std::cout << "\n----------- Test: MapVelocityToQDot --------------";
  std::cout << "\n--------------------------------------------------";
  std::cout << "\nquatDt_map =      \n" << quatDt_map;
  std::cout << "\nquatDt_NB_exact = \n" << quatDt_NB_exact;
  std::cout << "\n\nxyzDt_map=      \n" << xyzDt_map;
  std::cout << "\nxyzDt_exact =     \n" << xyzDt_exact;
  std::cout << "\n--------------------------------------------------\n";
#endif

  EXPECT_TRUE(CompareMatrices(xyzDt_map,      xyzDt_exact, tolerance));
  EXPECT_TRUE(CompareMatrices(quatDt_map, quatDt_NB_exact, tolerance));
}

//-----------------------------------------------------------------------------
// TODO(Mitiguy) Remove function unless useful for more than Evan's PR# 4604.
// If more generally useful, create Doxygen for this function.
//-----------------------------------------------------------------------------
void  TestMapQDotToVelocity(
    const drake::systems::RigidBodyPlant<double>& rigid_body_plant,
    const drake::systems::Context<double>& context,
    const Vector4d& quatDt_NB_exact,
    const Vector3d& xyzDt_exact,
    const Vector3d& w_NB_B_exact,
    const Vector3d& v_NBo_B_exact,
    const double tolerance) {

  // Form matrix of time-derivative of coordinates.
  Eigen::VectorXd coordinatesDt(7);
  coordinatesDt << xyzDt_exact, quatDt_NB_exact;

  // Test whether MapQDotToVelocity accurately converts time-derivative of
  // coordinates to motion variables.
  systems::BasicVector<double> wv_from_map(6);
  rigid_body_plant.MapQDotToVelocity(context, coordinatesDt, &wv_from_map);
  const Vector3d w_map(wv_from_map[0], wv_from_map[1], wv_from_map[2]);
  const Vector3d v_map(wv_from_map[3], wv_from_map[4], wv_from_map[5]);
#if 0  // TODO(mitiguy) Remove these debug statements.
  std::cout << "\n\n--------------------------------------------------";
  std::cout << "\n----------- Test: MapQDotToVelocity --------------";
  std::cout << "\n--------------------------------------------------";
  std::cout << "\nw_map         = \n" << w_map;
  std::cout << "\nw_NB_B_exact  = \n" << w_NB_B_exact;
  std::cout << "\n\nv from map  = \n" << v_map;
  std::cout << "\nv_NBo_B_exact = \n" << v_NBo_B_exact;
  std::cout << "\n--------------------------------------------------\n";
#endif

  EXPECT_TRUE(CompareMatrices(w_map,  w_NB_B_exact, tolerance));
  EXPECT_TRUE(CompareMatrices(v_map, v_NBo_B_exact, tolerance));
}


/**
 * Calculates exact solutions for quaternion, angular velocity, and angular
 * acceleration expressed in body-frame, for torque-free rotational motion of an
 * axis-symmetric rigid body B (uniform cylinder) in Newtonian frame (World) A,
 * where torque-free means the moment of forces about B's mass center is zero.
 * Right-handed orthogonal unit vectors Ax, Ay, Az fixed in A are initially
 * equal to right-handed orthogonal unit vectors Bx, By, Bz fixed in B,
 * where Bz is parallel to B's symmetry axis.
 * Note: The function CalculateExactRotationalSolutionNB() is a more general
 * solution that allows for initial misalignment of Bx, By, Bz.
 * Algorithm from [Kane, 1983] Sections 1.13 and 3.1, Pages 60-62 and 159-169.
 * @param t Current value of time.
 * @param w_NB_B_initial  B's initial angular velocity in N, expressed in B.
 * @returns Machine-precision values at time t are returned as defined below.
 *
 * std::tuple | Description
 * -----------|-------------------------------------------------
 * quat_AB    | Quaternion relating Ax, Ay, Az to Bx, By, Bz.
 *            | Note: quat_AB is analogous to the rotation matrix R_AB.
 * w_NB_B     | B's angular velocity in N, expressed in B, e.g., [wx, wy, wz].
 * wDt_NB_B   | B's angular acceleration in N, expressed in B, [wx', wy', wz'].
 *
 * - [Kane, 1983] "Spacecraft Dynamics," McGraw-Hill Book Co., New York, 1983.
 *   (with P. W. Likins and D. A. Levinson).  Available for free .pdf download:
 *   https://ecommons.cornell.edu/handle/1813/637
 */
std::tuple<Quaterniond, Vector3d, Vector3d>
CalculateExactRotationalSolutionABInitiallyAligned(const double t,
                                 const Vector3d& w_NB_B_initial) {
  using std::sin;
  using std::cos;
  using std::sqrt;

  // Constant values of moments of inertia.
  const double I = 0.04;
  const double J = 0.02;

  // Initial values of wx, wy, wz.
  const double wx0 = w_NB_B_initial[0];
  const double wy0 = w_NB_B_initial[1];
  const double wz0 = w_NB_B_initial[2];

  // Intermediate calculations for quaternion solution.
  const double p = sqrt(wx0 * wx0 + wy0 * wy0 + (wz0 * J / I) * (wz0 * J / I));
  const double s = (I - J) / I * wz0;
  const double coef = wz0 * (J / (I * p));
  const double spt2 = sin(p * t / 2);
  const double cpt2 = cos(p * t / 2);
  const double sst2 = sin(s * t / 2);
  const double cst2 = cos(s * t / 2);

  // Kane's analytical solution is for quaternion quat_AB that relates A to B,
  // where A is a set Ax, Ay, Az of right-handed orthogonal unit vectors which
  // are fixed in N (Newtonian frame/World) and initially aligned to Bx, By, Bz,
  // where Bz is parallel to B's symmetry axis.
  // Kane produced an analytical solution for quat_AB [eAB0, eAB1, eB2, eB3],
  // which allows for direct calculation of the R_AB rotation matrix.
  const double eAB1 = spt2 / p * (wx0 * cst2 + wy0 * sst2);
  const double eAB2 = spt2 / p * (-wx0 * sst2 + wy0 * cst2);
  const double eAB3 =  coef * spt2 * cst2 + cpt2 * sst2;
  const double eAB0 = -coef * spt2 * sst2 + cpt2 * cst2;
  const Quaterniond quat_AB(eAB0, eAB1, eAB2, eAB3);

  // Analytical solution for wx(t), wy(t), wz(t).
  const double wx =  wx0 * cos(s * t) + wy0 * sin(s * t);
  const double wy = -wx0 * sin(s * t) + wy0 * cos(s * t);
  const double wz =  wz0;

  // Analytical solution for time-derivatives of wx, wy, wz.
  const double wxDt =  (1 - J / I) * wy * wz;
  const double wyDt = (-1 + J / I) * wx * wz;
  const double wzDt = 0.0;

  // Create a tuple to package for returning.
  std::tuple<Quaterniond, Vector3d, Vector3d> returned_tuple;
  std::get<0>(returned_tuple) = quat_AB;
  Vector3d& w_NB_B   = std::get<1>(returned_tuple);
  Vector3d& wDt_NB_B = std::get<2>(returned_tuple);

  // Fill returned_tuple with rotation results.
  w_NB_B   << wx, wy, wz;
  wDt_NB_B << wxDt, wyDt, wzDt;

  return returned_tuple;
}


/**
 * Calculates exact solutions for quaternion and angular velocity expressed in
 * body-frame, and their time derivatives for torque-free rotational motion of
 * axis-symmetric rigid body B (uniform cylinder) in Newtonian frame (World) N,
 * where torque-free means the moment of forces about B's mass center is zero.
 * The quaternion characterizes the orientiation between right-handed orthogonal
 * unit vectors Nx, Ny, Nz fixed in N and right-handed orthogonal unit vectors
 * Bx, By, Bz fixed in B, where Bz is parallel to B's symmetry axis.
 * Note: CalculateExactRotationalSolutionABInitiallyAligned() implements the
 * algorithm from [Kane, 1983] Sections 1.13 and 3.1, Pages 60-62 and 159-169.
 * This function allows for initial misalignment of Nx, Ny, Nz and Bx, By, Bz.
 * @param t Current value of time.
 * @param quat_NB_initial Initial value of the quaternion (which should already
 *   be normalized) that relates Nx, Ny, Nz to Bx, By, Bz.
 *   Note: quat_NB_initial is analogous to the initial rotation matrix R_NB.
 * @param w_NB_B_initial  B's initial angular velocity in N, expressed in B.
 * @returns Machine-precision values at time t are returned as defined below.
 *
 * std::tuple | Description
 * -----------|-------------------------------------------------
 * quat_NB    | Quaternion relating Nx, Ny, Nz to Bx, By, Bz: [e0, e1, e2, e3].
 *            | Note: quat_NB is analogous to the rotation matrix R_NB.
 * quatDt     | Time-derivative of `quat_NB', i.e., [e0', e1', e2', e3'].
 * w_NB_B     | B's angular velocity in N, expressed in B, e.g., [wx, wy, wz].
 * wDt_NB_B   | B's angular acceleration in N, expressed in B, [wx', wy', wz'].
 *
 * - [Kane, 1983] "Spacecraft Dynamics," McGraw-Hill Book Co., New York, 1983.
 *   (with P. W. Likins and D. A. Levinson).  Available for free .pdf download:
 *   https://ecommons.cornell.edu/handle/1813/637
 */
std::tuple<Quaterniond, Vector4d, Vector3d, Vector3d>
   CalculateExactRotationalSolutionNB(const double t,
                                      const Quaterniond& quat_NB_initial,
                                      const Vector3d& w_NB_B_initial) {
  // Kane's analytical solution is for quaternion quat_AB that relates A to B,
  // where A is another set Ax, Ay, Az of right-handed orthogonal unit vectors
  // which are fixed in N (Newtonian frame) but initially aligned to Bx, By, Bz.
  Quaterniond quat_AB;
  Vector3d w_NB_B, wDt_NB_B;
  std::tie(quat_AB, w_NB_B, wDt_NB_B) =
      CalculateExactRotationalSolutionABInitiallyAligned(t, w_NB_B_initial);

  // Multiply (Hamilton product) the quaternions analogous to the rotation
  // matrices R_NA and R_AB to produce the quaternion characterizing R_NB.
  // In other words, account for quat_NB_initial (which is quat_NA) to calculate
  // the quaternion quat_NB that is analogous to the R_NB rotation matrix.

  // Define A basis as World-fixed basis aligned with B's initial orientation.
  const Quaterniond& quat_NA = quat_NB_initial;
  const Quaterniond quat_NB = quat_NA * quat_AB;

  // Analytical solution for time-derivative quaternion in B.
  const Vector4d quatDt = CalculateQuaternionDtFromAngularVelocityExpressedInB(
                          quat_NB, w_NB_B);

  // Create a tuple to package for returning.
  std::tuple<Quaterniond, Vector4d, Vector3d, Vector3d> returned_tuple;
  std::get<0>(returned_tuple) = quat_NB;
  std::get<1>(returned_tuple) = quatDt;
  std::get<2>(returned_tuple) = w_NB_B;
  std::get<3>(returned_tuple) = wDt_NB_B;

  return returned_tuple;
}


/**
 * Calculates exact solutions for translational motion of an arbitrary rigid
 * body B in a Newtonian frame (world) N.  Algorithm from high-school physics.
 * @param t Current value of time.
 * @param xyz_initial Initial values of x, y, z -- [the Nx, Ny, Nz measures of
 * Bcm's (B's center of mass) position from a point No fixed in World N].
 * Note: Nx, Ny, Nz are right-handed orthogonal unit vectors fixed in N.
 * @param xyzDt_initial Initial values of x', y', z' (which are time-derivatives
 * of x, y,z -- and equal to the Nx, Ny, Nz measures of Bcm's velocity in N).
 * @param gravity Gravitational acceleration expressed in N (e.g. [0, 0, -9.8]).
 * @returns Machine-precision values at time t are returned as defined below.
 *
 * std::tuple | Description
 * -----------|-----------------------------------------------------------
 * xyz        | Vector3d x, y, z (Bcm's position from No, expressed in N).
 * xyzDt      | Vector3d with first-time-derivative of x, y, z.
 * xyzDDt     | Vector3D with second-time-derivative of x, y, z.
 */
std::tuple<Vector3d, Vector3d, Vector3d>
    CalculateExactTranslationalSolution(const double t,
                                        const Vector3d& xyz_initial,
                                        const Vector3d& xyzDt_initial,
                                        const Vector3d& gravity) {
  // Initial values of x, y, z and x', y', z'.
  const double x0 = xyz_initial[0];
  const double y0 = xyz_initial[1];
  const double z0 = xyz_initial[2];
  const double xDt0 = xyzDt_initial[0];
  const double yDt0 = xyzDt_initial[1];
  const double zDt0 = xyzDt_initial[2];

  // Analytical solution for x'', y'', z'' (only gravitational forces on body).
  const double x_acceleration = gravity[0];
  const double y_acceleration = gravity[1];
  const double z_acceleration = gravity[2];

  // Analytical solution for x', y', z' (constant acceleration).
  const double xDt = xDt0 + x_acceleration * t;
  const double yDt = yDt0 + y_acceleration * t;
  const double zDt = zDt0 + z_acceleration * t;

  // Analytical solution for x, y, z (constant acceleration).
  const double x = x0 + xDt0*t + 0.5 * x_acceleration * t * t;
  const double y = y0 + yDt0*t + 0.5 * y_acceleration * t * t;
  const double z = z0 + zDt0*t + 0.5 * z_acceleration * t * t;

  // Create a tuple to package for returning.
  std::tuple<Vector3d, Vector3d, Vector3d> returned_tuple;
  Vector3d& xyz    = std::get<0>(returned_tuple);
  Vector3d& xyzDt  = std::get<1>(returned_tuple);
  Vector3d& xyzDDt = std::get<2>(returned_tuple);

  // Fill returned_tuple with rotation results.
  xyz    << x, y, z;
  xyzDt  << xDt, yDt, zDt;
  xyzDDt << x_acceleration, y_acceleration, z_acceleration;

  return returned_tuple;
}


/**
 * This function tests Drake state versus an exact solution for torque-free
 * motion of axis-symmetric uniform rigid cylinder B in Newtonian frame/World N,
 * where torque-free means the moment of forces about B's mass center is zero.
 * The quaternion characterizes the orientiation between right-handed orthogonal
 * unit vectors Nx, Ny, Nz fixed in N and right-handed orthogonal unit vectors
 * Bx, By, Bz fixed in B, where Bz is parallel to B's symmetry axis.
 * @param rigid_body_plant A reference to the rigid-body system being simulated.
 * @param[in] context All input to System (input ports, parameters, time, state)
 * with cache for computations dependent on these values.
 * @param[out] stateDt_drake On output, stores value at t = t_final.
 * @param quat_NB_initial Initial value of the quaternion (which should already
 *   be normalized) that relates Nx, Ny, Nz to Bx, By, Bz.
 *   Note: quat_NB_initial is analogous to the initial rotation matrix R_NB.
 * @param w_NB_B_initial  B's initial angular velocity in N, expressed in B.
 * @param xyz_initial Initial values of x, y, z -- [the Nx, Ny, Nz measures of
 * Bcm's (B's center of mass) position from a point No fixed in N].
 * @param v_NBo_B_initial Initial values of Bx, By, Bz measures of Bo's velocity
 * in N, where Bo is the same as Bcm.  Note: These values are not (in general)
 * x', y', z' (i.e., these values are not time-derivatives of x, y,z).
 * @param tolerance Minimum tolerance required to match results.
 */
  void TestDrakeSolutionVsExactSolutionForTorqueFreeCylinder(
       const drake::systems::RigidBodyPlant<double>& rigid_body_plant,
       const drake::systems::Context<double>& context,
       drake::systems::ContinuousState<double>* stateDt_drake,
       const Quaterniond& quat_NB_initial,
       const Vector3d& w_NB_B_initial,
       const Vector3d& xyz_initial,
       const Vector3d& v_NBo_B_initial,
       const double tolerance) {
  DRAKE_DEMAND(stateDt_drake != NULL);

  // Pull the state at time t from the context.
  const systems::VectorBase<double>& state_drake =
      context.get_continuous_state_vector();

  // Evaluate the time-derivatives of the state.
  rigid_body_plant.CalcTimeDerivatives(context, stateDt_drake);

  // Get the state (defined above) and state time-derivatives for comparison.
  const VectorXd state_as_vector = state_drake.CopyToVector();
  const Vector3d xyz_drake     = state_as_vector.segment<3>(0);
  const Vector4d quat_NB_drake = state_as_vector.segment<4>(3);
  const Vector3d w_NB_B_drake  = state_as_vector.segment<3>(7);
  const Vector3d v_NBo_B_drake = state_as_vector.segment<3>(10);

  const VectorXd stateDt_as_vector = stateDt_drake->CopyToVector();
  const Vector3d xyzDt_drake     = stateDt_as_vector.segment<3>(0);
  const Vector4d quatDt_NB_drake = stateDt_as_vector.segment<4>(3);
  const Vector3d wDt_NB_B_drake  = stateDt_as_vector.segment<3>(7);
  const Vector3d vDt_NBo_B_drake = stateDt_as_vector.segment<3>(10);

  // Reserve space for values returned by calculating exact solution.
  Quaterniond quatd_NB_exact;
  Vector4d quatDt_NB_exact;
  Vector3d w_NB_B_exact, wDt_NB_B_exact;
  Vector3d xyz_exact, xyzDt_exact, xyzDDt_exact;

  // Calculate exact analytical rotational solution.
  const double t = context.get_time();
  std::tie(quatd_NB_exact, quatDt_NB_exact, w_NB_B_exact, wDt_NB_B_exact) =
      CalculateExactRotationalSolutionNB(t, quat_NB_initial, w_NB_B_initial);

  // Get gravitational acceleration expressed in N (e.g. [0, 0, -9.8]).
  // Note: a_grav is an improperly-named public member of tree class (BAD).
  const RigidBodyTree<double>& tree = rigid_body_plant.get_rigid_body_tree();
  const Vector3d gravity = tree.a_grav.tail<3>();

  // Calculate exact analytical translational solution.
  // Exact analytical solution needs v_initial expressed in terms of Nx, Ny, Nz.
  const Eigen::Matrix3d R_NB_initial = quat_NB_initial.toRotationMatrix();
  const Vector3d v_NBo_N_initial = R_NB_initial * v_NBo_B_initial;
  std::tie(xyz_exact, xyzDt_exact, xyzDDt_exact) =
  CalculateExactTranslationalSolution(t, xyz_initial, v_NBo_N_initial, gravity);

  // Since more than one quaternion is associated with the same orientation,
  // compare Drake's canonical quaternion with exact canonical quaternion.
  const Quaterniond quatd_NB_drake = math::quat2eigenQuaternion(quat_NB_drake);
  const bool is_ok_quat = AreQuaternionsApproximatlyEqualInCanonicalForm(
                          quatd_NB_exact, quatd_NB_drake, tolerance);
  EXPECT_TRUE(is_ok_quat);

  // Convert quaternions to rotation matrix and compare rotation matrices.
  Vector4d quat_NB_exact;
  quat_NB_exact << quatd_NB_exact.w(), quatd_NB_exact.x(),
                   quatd_NB_exact.y(), quatd_NB_exact.z();
  const Eigen::Matrix3d R_NB_drake = math::quat2rotmat(quat_NB_drake);
  const Eigen::Matrix3d R_NB_exact = math::quat2rotmat(quat_NB_exact);
  EXPECT_TRUE(R_NB_drake.isApprox(R_NB_exact, tolerance));

  // Drake: Compensate for definition of vDt = acceleration - w x v.
  const Vector3d w_cross_v_drake = w_NB_B_drake.cross(v_NBo_B_drake);
  const Vector3d xyzDDt_drake = R_NB_drake *(vDt_NBo_B_drake + w_cross_v_drake);

  // Exact: Compensate for definition of vDt = acceleration - w x v.
  const Eigen::Matrix3d R_BN_exact = R_NB_exact.inverse();
  const Vector3d v_NBo_B_exact = R_BN_exact * xyzDt_exact;
  const Vector3d w_cross_v_exact = w_NB_B_exact.cross(v_NBo_B_exact);
  const Vector3d vDt_NBo_B_exact = R_BN_exact * xyzDDt_exact - w_cross_v_exact;

#if 0  // TODO(mitiguy) Remove these debug statements.
  std::cout << "\n\n quat_NB_drake\n"   << quat_NB_drake;
  std::cout << "\n quat_NB_exact\n"     << quat_NB_exact;
  std::cout << "\n\n quatDt_drake\n"    << quatDt_NB_drake;
  std::cout << "\n quatDt_NB_exact\n"   << quatDt_NB_exact;
  std::cout << "\n\n w_NB_B_drake\n"    << w_NB_B_drake;
  std::cout << "\n w_NB_B_exact\n"      << w_NB_B_exact;
  std::cout << "\n\n wDt_NB_B_drake\n"  << wDt_NB_B_drake;
  std::cout << "\n wDt_NB_B_exact\n"    << wDt_NB_B_exact;
  std::cout << "\n\n xyz_drake\n"       << xyz_drake;
  std::cout << "\n xyz_exact\n"         << xyz_exact;
  std::cout << "\n\n xyzDt_drake\n"     << xyzDt_drake;
  std::cout << "\n xyzDt_exact\n"       << xyzDt_exact;
  std::cout << "\n\n xyzDDt_drake\n"    << xyzDDt_drake;
  std::cout << "\n xyzDDt_exact\n"      << xyzDDt_exact;
  std::cout << "\n\n v_NBo_B_drake\n"   << v_NBo_B_drake;
  std::cout << "\n v_NBo_B_exact\n"     << v_NBo_B_exact;
  std::cout << "\n\n vDt_NBo_B_drake\n" << vDt_NBo_B_drake;
  std::cout << "\n vDt_NBo_B_exact\n"   << vDt_NBo_B_exact << "\n\n";
#endif

  // Compare Drake and exact results.
  // TODO(mitiguy) Investigate why big factor (1600) needed to test wDt_NB_B.
  EXPECT_TRUE(CompareMatrices(w_NB_B_drake,       w_NB_B_exact,     tolerance));
  EXPECT_TRUE(CompareMatrices(wDt_NB_B_drake,  wDt_NB_B_exact, 1600*tolerance));
  EXPECT_TRUE(CompareMatrices(xyz_drake,             xyz_exact,     tolerance));
  EXPECT_TRUE(CompareMatrices(xyzDt_drake,         xyzDt_exact,     tolerance));
  EXPECT_TRUE(CompareMatrices(xyzDDt_drake,       xyzDDt_exact,  10*tolerance));
  EXPECT_TRUE(CompareMatrices(v_NBo_B_drake,     v_NBo_B_exact,     tolerance));
  EXPECT_TRUE(CompareMatrices(vDt_NBo_B_drake, vDt_NBo_B_exact,  10*tolerance));

  // Multi-step process to compare Drake vs exact time-derivative of quaternion.
  // Ensure time-derivative of Drake's quaternion satifies quarternionDt test.
  // Since more than one time-derivative of a quaternion is associated with the
  // same angular velocity, convert to angular velocity to compare results.
  EXPECT_TRUE(IsBothQuaternionAndQuaternionDtOK(quatd_NB_drake,
                                                quatDt_NB_drake, 16*tolerance));
  const Vector3d w_from_quatDt_drake =
      CalculateAngularVelocityExpressedInBFromQuaternionDt(
          quatd_NB_drake, quatDt_NB_drake);


  const Vector3d w_from_quatDt_exact =
      CalculateAngularVelocityExpressedInBFromQuaternionDt(
          quatd_NB_exact, quatDt_NB_exact);
  EXPECT_TRUE(CompareMatrices(w_from_quatDt_drake, w_NB_B_drake, tolerance));
  EXPECT_TRUE(CompareMatrices(w_from_quatDt_exact, w_NB_B_exact, tolerance));


  //--------------------------------------------------------------
  // EXTRA: Test MapQDotToVelocity and MapVelocityToQDot for Evan's PR #4604.
  //--------------------------------------------------------------
  TestMapVelocityToQDot(rigid_body_plant, context, w_NB_B_exact, v_NBo_B_exact,
                        quatDt_NB_exact, xyzDt_exact, tolerance);

  TestMapQDotToVelocity(rigid_body_plant, context, quatDt_NB_exact, xyzDt_exact,
                        w_NB_B_exact, v_NBo_B_exact, tolerance);
}


/**
 * Numerically integrate rigid body plant's equations of motion from time t = 0
 * to time t_final with a 3rd-order variable-step Runge-Kutta integrator.
 * @param[in] rigid_body_plant
 *    A reference to the rigid-body system being simulated.
 * @param[in,out] context
 *    On entry, context should be properly filled with all input to System
 *    (input ports, parameters, time = 0.0, state).  On output, context has been
 *    changed so its time = t_final and its state has been updated.
 * @param[in] dt
 *    The maximum (fixed) step size taken by the integrator.  If  t_final
 *    is not an exact multiple of dt, the final integration step is adjusted.
 * @param[in] t_final
 *    Value of time that signals the simulation to stop.
 * @param[in] maximum_absolute_error_per_integration_step
 *    Maximum allowable absolute error (for any variable being integrated) for
 *    the numerical-integrator's internal time-step (which can shrink or grow).
 */
void  IntegrateForwardWithVariableStepRungeKutta3(
    const drake::systems::RigidBodyPlant<double>& rigid_body_plant,
    drake::systems::Context<double>* context,
    double dt, const double t_final,
    const double maximum_absolute_error_per_integration_step) {
  DRAKE_DEMAND(context != NULL  &&  dt >= 0.0);

  // The initial value of time t is set from the context.
  double t = context->get_time();
  if ( t < t_final ) {
    // Integrate with variable-step Runge-Kutta3 integrator.
    systems::RungeKutta3Integrator<double> rk3(rigid_body_plant, context);
    rk3.set_maximum_step_size(dt);  // Needed before Initialize (or exception).
    rk3.set_target_accuracy(maximum_absolute_error_per_integration_step);
    rk3.Initialize();

    // Integrate to within a small amount of dt of t_final.
    const double epsilon_based_on_dt = 1.0E-9 * dt;
    const double t_final_minus_epsilon = t_final - epsilon_based_on_dt;
    do {
      if ( t + dt > t_final ) dt = t_final - t;
      rk3.StepOnceAtMost(dt, dt, dt);    // Step forward by dt.
      t += dt;                           // or t = context->get_time();
    } while ( t < t_final_minus_epsilon );
  }
}


/** This function tests Drake's calculation of the time-derivative of the state
 * at many initial values. For certain initial values (somewhat randomly chosen)
 * this function also tests Drake's numerical integration of the rigid body's
 * equations of motion.
 * @param[in] rigid_body_plant
 *    A reference to the rigid-body system being simulated.
 * @param[in,out] context
 *    On entry, context is properly sized for all input to System (input ports,
 *    parameters, time, state).  On output, context may be changed with
 *    specific values of time and state.
 * @param[out] stateDt_drake
 *    On output, stores time-derivatives of state.
 * @param[in] quat_NB_initial
 *   Initial value of the quaternion (which should already be normalized) that
 *   that relates Nx, Ny, Nz to Bx, By, Bz.
 *   Note: quat_NB_initial is analogous to the initial rotation matrix R_NB.
 * @param[in] w_NB_B_initial
 *   B's initial angular velocity in N, expressed in B.
 * @param[in] xyz_initial
 *   Initial values of x, y, z -- [the Nx, Ny, Nz measures of Bcm's (B's center
 *   of mass) position from a point No fixed in N (NewtonianFrame/World N)].
 * @param[in] v_NBo_B_initial
 *    Initial values of Bx, By, Bz measures of Bo's velocity in N, where Bo is
 *    Bcm.  Note: These values are not (in general) x', y', z' (i.e., these
 *    values are not time-derivatives of x, y,z).
 * @param[in] should_numerically_integrate_one_second
 *    True if also simulate 1 second and check accuracy of simulation results.
 */
void  TestDrakeSolutionForSpecificInitialValue(
    const drake::systems::RigidBodyPlant<double>& rigid_body_plant,
    drake::systems::Context<double>* context,
    drake::systems::ContinuousState<double>* stateDt_drake,
    const Eigen::Quaterniond& quat_NB_initial,
    const Vector3d& w_NB_B_initial,
    const Vector3d& xyz_initial,
    const Vector3d& v_NBo_B_initial,
    const bool should_numerically_integrate_one_second) {
  DRAKE_DEMAND(context != NULL  &&  stateDt_drake != NULL);

  // Get the state portion of the Context. State has unusual order/mearning.
  // State for joints::kQuaternion -- x,y,z, e0,e1,e2,e3, wx,wy,wz, vx,vy,vz.
  // Note: Bo is the origin of rigid cylinder B (here, coincident with Bcm).
  // Bx, By, Bz are fixed in B, with Bz along cylinder B's symmetric axis.
  // No is the origin of the Newtonian reference frame (World) N.
  // Nx, Ny, Nz are fixed in N, with Nz vertically upward (opposite gravity).
  // e0, e1, e2, e3 is the quaternion relating Nx, Ny, Nz to Bx, By, Bz,
  // with e0 being the scalar term in the quaternion.
  // w_NB_B is B's angular velocity in N expressed in B, [wx, wy, wz].
  // wDt_B is  B's angular acceleration in N, expressed in B, [wx', wy', wz'].
  // x, y, z are Bo's position from No, expressed in N.
  // v_NBo_B is Bo's velocity in N, expressed in B: [vx,vy,vz] not [x', y', z'].
  // vDt_B is the time-derivative in B of v_NBo_B, [vx', vy', vz'] - which is
  // not physically meaningful. Note: Bo's acceleration in N, expressed in B, is
  // calculated by [Kane, 1985, pg. 23], as: a_NBo_B = vDt_B + w_NB_B x v_NBo_B.
  //
  // - [Kane, 1985] "Dynamics: Theory and Applications," McGraw-Hill Book Co.,
  //   New York, 1985 (with D. A. Levinson).  Available for free .pdf download:
  //  https://ecommons.cornell.edu/handle/1813/637
  //
  // TODO(mitiguy) Update comment/code when GitHub issue #4398 is fixed.
  // Concatenate these 4 Eigen column matrices into one Eigen column matrix.
  // Note: The state has a weird order (see previous comment).
  // Set state portion of Context from initial state.
  Eigen::Matrix<double, 13, 1> state_initial;
  state_initial << xyz_initial,
                   quat_NB_initial.w(),
                   quat_NB_initial.x(),
                   quat_NB_initial.y(),
                   quat_NB_initial.z(),
                   w_NB_B_initial,
                   v_NBo_B_initial;
  systems::VectorBase<double>& state_drake =
      *(context->get_mutable_continuous_state_vector());
  state_drake.SetFromVector(state_initial);

  // Ensure the time stored by context is set to 0.0 (initial value).
  context->set_time(0.0);

  // Test Drake's calculated values for time-derivative of state at time t = 0
  // versus exact analytical (closed-form) solution.
  // Initially, (time=0), Drake's results should be close to machine-precision.
  const double epsilon = std::numeric_limits<double>::epsilon();
  TestDrakeSolutionVsExactSolutionForTorqueFreeCylinder(
                      rigid_body_plant, *context, stateDt_drake,
                      quat_NB_initial, w_NB_B_initial,
                      xyz_initial, v_NBo_B_initial, 50 * epsilon);

  // Maybe numerically integrate with one of Drake's numerical integrators.
  if ( should_numerically_integrate_one_second ) {
    const double dt = 0.2, t_final = 10.0;
    const double maximum_absolute_error_per_integration_step = 1.0E-6;
    IntegrateForwardWithVariableStepRungeKutta3(rigid_body_plant, context,
                 dt, t_final, maximum_absolute_error_per_integration_step);

    // After numerically integrating to t = t_final, test Drake's simulation
    // accuracy at t = t_final versus exact analytical (closed-form) solution.
    const double tolerance = maximum_absolute_error_per_integration_step;
    TestDrakeSolutionVsExactSolutionForTorqueFreeCylinder(
                        rigid_body_plant, *context, stateDt_drake,
                        quat_NB_initial, w_NB_B_initial,
                        xyz_initial, v_NBo_B_initial, tolerance);
  }
}


/** This function tests Drake's calculation of the time-derivative of the state
 * at many initial values. For certain initial values (somewhat randomly chosen)
 * this function also tests Drake's numerical integration of the rigid body's
 * equations of motion.
 * @param[in] rigid_body_plant
 *    A reference to the rigid-body system being simulated.
 * @param[in,out] context
 *    On entry, context is properly sized for all input to System (input ports,
 *    parameters, time, state).  On output, context may be changed with
 *    specific values of time and state.
 * @param[out] stateDt_drake
 *    On output, stores time-derivatives of state.
 */
void  TestDrakeSolutionForVariousInitialValues(
      const drake::systems::RigidBodyPlant<double>& rigid_body_plant,
      drake::systems::Context<double>* context,
      drake::systems::ContinuousState<double>* stateDt_drake) {
  DRAKE_DEMAND(context != NULL  &&  stateDt_drake != NULL);

  // Create 3x1 matrix for wx, wy, wz (defined above).
  // Create 3x1 matrix for x, y, z (defined above).
  // Create 3x1 matrix for vx, vy, vz (defined above -- not x', y', z').
  const Vector3d w_NB_B_initial(2.0, 4.0, 6.0);
  const Vector3d xyz_initial(1.0, 2.0, 3.0);
  const Vector3d v_NBo_B_initial(-4.2, 5.5, 6.1);

  // Create 4x1 matrix for normalized quaternion e0, e1, e2, e3 (defined above).
  // Iterate through many initial values for quaternion.
  // Since cylinder B is axis-symmetric for axis Bz, iterate on BodyXY rotation
  // sequence with 0 <= thetaX <= 2*pi and 0 <= thetaY <= pi.
  unsigned int test_counter = 0u;
  for (double thetaX = 0; thetaX <= 2*M_PI; thetaX += 0.02*M_PI) {
    for (double thetaY = 0; thetaY <= M_PI; thetaY += 0.05*M_PI) {
      const Vector3<double> spaceXYZ_angles = {thetaX, thetaY, 0};
      const Vector4d vector4d_NB_initial = math::rpy2quat(spaceXYZ_angles);
      Eigen::Quaterniond quat_NB_initial(vector4d_NB_initial(0),
                                         vector4d_NB_initial(1),
                                         vector4d_NB_initial(2),
                                         vector4d_NB_initial(3));

      // Since there are 100*20 = 2000 total tests of initial conditions, it is
      // time-consuming to numerically integrate for every initial condition.
      // Hence, numerical integration is only tested sporadically.
      const bool also_numerically_integrate = (test_counter++ % 210 == 0);
      TestDrakeSolutionForSpecificInitialValue(rigid_body_plant,
                                               context,
                                               stateDt_drake,
                                               quat_NB_initial,
                                               w_NB_B_initial,
                                               xyz_initial,
                                               v_NBo_B_initial,
                                               also_numerically_integrate);
    }
  }
}


/**
 * This function tests Drake's simulation of the motion of an axis-symmetric
 * rigid body B (uniform cylinder) in a Newtonian frame (World) N.  Since the
 * only external forces on B are uniform gravitational forces, there exists
 * an exact closed-form analytical solution for B's motion.  The closed-form
 * rotational solution is available since B is "torque-free", i.e., the moment
 * of all forces about B's mass center is zero.
 * Specifically, this function tests Drake's solution for quaternion, angular
 * velocity and angular acceleration expressed in B (body-frame).  This function
 * also tests Drake's solution for position, velocity, and acceleration
 * expressed in N (World).
 * Algorithm from [Kane, 1983] Sections 1.13 and 3.1, Pages 60-62 and 159-169.
 *
 * - [Kane, 1983] "Spacecraft Dynamics," McGraw-Hill Book Co., New York, 1983.
 *   (with P. W. Likins and D. A. Levinson).  Available for free .pdf download:
 *   https://ecommons.cornell.edu/handle/1813/637
 */
// Test Drake solution versus closed-form analytical solution.
GTEST_TEST(uniformSolidCylinderTorqueFree, testA) {
  // Create path to .urdf file containing mass/inertia and geometry properties.
  const std::string urdf_name = "uniform_solid_cylinder.urdf";
  const std::string urdf_dir = "/multibody/benchmarks/"
      "cylinder_torque_free_analytical_solution/";
  const std::string urdf_dir_file_name = GetDrakePath() + urdf_dir + urdf_name;

  // Construct an empty RigidBodyTree.
  std::unique_ptr<RigidBodyTree<double>> tree =
      std::make_unique<RigidBodyTree<double>>();

  // Populate RigidBodyTree tree by providing path to .urdf file,
  // second argument is enum FloatingBaseType that specifies the joint
  // connecting the robot's base link to ground/world. enum values are:
  // kFixed         welded to ground.
  // kRollPitchYaw  translates x,y,z and rotates 3D (by SpaceXYZ Euler angles).
  // kQuaternion    translates x,y,z and rotates 3D (by Euler parameters).
  const drake::multibody::joints::FloatingBaseType joint_type =
      drake::multibody::joints::kQuaternion;
  std::shared_ptr<RigidBodyFrame<double>> weld_to_frame = nullptr;
  drake::parsers::urdf::AddModelInstanceFromUrdfFile(
      urdf_dir_file_name, joint_type, weld_to_frame, tree.get());

  // Create a RigidBodyPlant which takes ownership of tree.
  drake::systems::RigidBodyPlant<double> rigid_body_plant(std::move(tree));

  // Create a Context which stores state and extra calculations.
  std::unique_ptr<systems::Context<double>> context =
      rigid_body_plant.CreateDefaultContext();

  // Allocate space to hold time-derivative of state_drake.
  std::unique_ptr<systems::ContinuousState<double>> stateDt_drake =
      rigid_body_plant.AllocateTimeDerivatives();

  TestDrakeSolutionForVariousInitialValues(rigid_body_plant, context.get(),
                                           stateDt_drake.get());
}

}  // namespace cylinder_torque_free_analytical_solution
}  // namespace benchmarks
}  // namespace drake
