// Purpose: Tests various quaternion functions and methods.
//-----------------------------------------------------------------------------
#include <cmath>
#include <gtest/gtest.h>

#include "drake/common/drake_assert.h"
#include "drake/common/eigen_matrix_compare.h"
#include "drake/math/autodiff.h"
#include "drake/math/quaternion.h"


namespace drake {
namespace math {
namespace quaternion_test {

// This tests is_quaternion_in_canonical_form, i.e., whether a quaternion is in
// canonical form.  For [e0, e1, e2, e3], it tests whether e0 is non-negative.
GTEST_TEST(is_quaternion_in_canonical_form, testA) {
  const double half_angle = M_PI/6;
  const double cos_half_angle = std::cos(half_angle);
  const double sin_half_angle = std::sin(half_angle);
  const double x = 0.3, y = 0.7, z = std::sqrt(1.0 - (x*x + y*y));
  const double e0 = cos_half_angle;
  const double e1 = sin_half_angle * x;
  const double e2 = sin_half_angle * y;
  const double e3 = sin_half_angle * z;
  const Eigen::Quaterniond a(+e0, +e1, +e2, +e3);
  const Eigen::Quaterniond b(-e0, -e1, -e2, -e3);
  EXPECT_TRUE(is_quaternion_in_canonical_form(a));
  EXPECT_FALSE(is_quaternion_in_canonical_form(b));
}


// This tests QuaternionToCanonicalForm (conversion of a quaternion to canonical
// form. For quaternion [e0, e1, e2, e3] = [-0.3, +0.4, +0.5, +0.707], it tests
// that QuaternionToCanonicalForm returns  [+0.3, -0.4, -0.5, -0.707].
GTEST_TEST(QuaternionToCanonicalFormTest, testA) {
  const double half_angle = M_PI/6;
  const double cos_half_angle = std::cos(half_angle);
  const double sin_half_angle = std::sin(half_angle);
  const double x = 0.3, y = 0.7, z = std::sqrt(1.0 - (x*x + y*y));
  const double e0 = cos_half_angle;
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


// This tests function AreQuaternionsApproximatlyEqualInCanonicalForm (whether
// two quaternions are approximately equal when both are put in canonical form).
GTEST_TEST(AreQuaternionsApproximatlyEqualInCanonicalFormTest, testA) {
  const double half_angle = M_PI/6;
  const double cos_half_angle = std::cos(half_angle);
  const double sin_half_angle_a = std::sin(half_angle);
  const double x = 0.3, y = 0.7, z = std::sqrt(1.0 - (x*x + y*y));
  const double e0a = cos_half_angle;
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


// This function tests CalculateQuaternionDtFromAngularVelocityExpressedInB.
GTEST_TEST(CalculateQuaternionDtFromAngularVelocityExpressedInBTest, testA) {
  const double half_angle = M_PI/6;
  const double cos_half_angle = std::cos(half_angle);
  const double sin_half_angle = std::sin(half_angle);
  const double x = 0.3, y = 0.7, z = std::sqrt(1.0 - (x*x + y*y));

  // For convenience, locally typedef Vector7d and AutoDiff7d.
  using Vector7d = Eigen::Matrix<double, 7, 1>;
  using AutoDiff7d = Eigen::AutoDiffScalar<Vector7d>;

  // Initialize each variable that is to be regarded as an independent variable
  // (for purposes of partial differentiation) with its numerical value (i.e.,
  // (where the function and its nth derivatives are to be evaluated) and the
  // appropriate entry for it in the 7d array.
  const AutoDiff7d e0(cos_half_angle,       7, 0);
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


// This function tests CalculateQuaternionDtFromAngularVelocityExpressedInB.
GTEST_TEST(CalculateAngularVelocityExpressedInBFromQuaternionDtTest, testA) {
  const double half_angle = M_PI/6;
  const double cos_half_angle = std::cos(half_angle);
  const double sin_half_angle = std::sin(half_angle);
  const double x = 0.3, y = 0.7, z = std::sqrt(1.0 - (x*x + y*y));

  // For convenience, locally typedef Vector7d and AutoDiff7d.
  using Vector8d = Eigen::Matrix<double, 8, 1>;
  using AutoDiff8d = Eigen::AutoDiffScalar<Vector8d>;

  // Initialize each variable that is to be regarded as an independent variable
  // (for purposes of partial differentiation) with its numerical value (i.e.,
  // (where the function and its nth derivatives are to be evaluated) and the
  // appropriate entry for it in the 7d array.
  // e0Dt, e1Dt, e2Dt are initialized with arbitrary values of 0, 1, 2, and
  // e3Dt is initialized so it satisfies: e0*e0' + e1*e1' + e2*e2' + e3*e3' = 0.
  const AutoDiff8d e0(cos_half_angle,      8, 0);
  const AutoDiff8d e1(sin_half_angle * x,  8, 1);
  const AutoDiff8d e2(sin_half_angle * y,  8, 2);
  const AutoDiff8d e3(sin_half_angle * z,  8, 3);
  const AutoDiff8d e0Dt(0.0,                 8, 4);
  const AutoDiff8d e1Dt(1.0,                 8, 5);
  const AutoDiff8d e2Dt(2.0,                 8, 6);
  const AutoDiff8d e3Dt(-2.623156949355562,  8, 7);
  const Eigen::Quaternion<AutoDiff8d> quat(e0,  e1,  e2,  e3);
  const drake::Vector4<AutoDiff8d> quatDt(e0Dt, e1Dt, e2Dt, e3Dt);
  const drake::Vector3<AutoDiff8d> w_B =
      CalculateAngularVelocityExpressedInBFromQuaternionDt(quat, quatDt);

  // Verify Drake's angular velocity vs near-exact MotionGenesis (MG) results.
  const double wx_MG = 4.864408811799343;
  const double wy_MG = 2.029080460490301;
  const double wz_MG = -4.443441112511214;
  const Eigen::Vector3d w_B_MG(wx_MG, wy_MG, wz_MG);
  const Eigen::Vector3d w_B_numerical = math::autoDiffToValueMatrix(w_B);
  const double epsilon = std::numeric_limits<double>::epsilon();
  EXPECT_TRUE(CompareMatrices(w_B_numerical, w_B_MG, 8*epsilon));

  // Test Drake partials of angular velocity with respect to
  // [e0, e1, e2, e3, e0', e1', e2', e3'].
  const Vector8d wx_partials = w_B(0).derivatives();
  const Vector8d wy_partials = w_B(1).derivatives();
  const Vector8d wz_partials = w_B(2).derivatives();

  // MotionGenesis was used to calculate and evaluate same partial derivatives.
  Vector8d wx_partial_MG, wy_partial_MG, wz_partial_MG;
  wx_partial_MG << 2,              0,                 5.246313898711124,  4,
                  -0.3,            1.732050807568877, 0.648074069840786, -0.7;
  wy_partial_MG << 4,             -5.246313898711124, 0,                 -2,
                  -0.7,           -0.648074069840786, 1.732050807568877,  0.3;
  wz_partial_MG << -5.246313898711124, -4,            2,                    0,
                   -0.648074069840786,  0.7,         -0.3,  1.732050807568877;

  // Verify Drake's partials are nearly identical to exact values.
  EXPECT_TRUE(CompareMatrices(wx_partials, wx_partial_MG, 8*epsilon));
  EXPECT_TRUE(CompareMatrices(wy_partials, wy_partial_MG, 8*epsilon));
  EXPECT_TRUE(CompareMatrices(wz_partials, wz_partial_MG, 8*epsilon));
}


// This function tests CalculateQuaternionDtConstraintViolation.
GTEST_TEST(CalculateQuaternionDtConstraintViolationTest, testA) {
  const double half_angle = M_PI/6;
  const double cos_half_angle = std::cos(half_angle);
  const double sin_half_angle = std::sin(half_angle);
  const double x = 0.3, y = 0.7, z = std::sqrt(1.0 - (x*x + y*y));

  // MotionGenesis was used to calculate values for e0Dt, e1Dt, e2Dt, e3Dt that
  // satisifies the constraint  e0*e0' + e1*e1' + e2*e2' + e3*e3' = 0.
  const double e0 = cos_half_angle;
  const double e1 = sin_half_angle * x;
  const double e2 = sin_half_angle * y;
  const double e3 = sin_half_angle * z;
  const double e0Dt = -0.4211981425390414;
  const double e1Dt =  2.387689633338328;
  const double e2Dt = -1.529504299767133;
  const double e3Dt =  1.672467320028763;
  const Eigen::Quaterniond quat(e0,  e1,  e2,  e3);
  const drake::Vector4<double> quatDt(e0Dt, e1Dt, e2Dt, e3Dt);
  const drake::Vector4<double> quatDt_bad(quatDt[0], quatDt[1], quatDt[2],
                                          0.99*quatDt[3]);
  const double a = CalculateQuaternionDtConstraintViolation(quat, quatDt);
  const double b = CalculateQuaternionDtConstraintViolation(quat, quatDt_bad);

  // For the given numbers, a should be near zero, whereas b is not.
  const double epsilon = std::numeric_limits<double>::epsilon();
  EXPECT_TRUE(std::abs(a) <=  8*epsilon);
  EXPECT_TRUE(std::abs(b) >=  0.01);
}


// This function tests CalculateQuaternionDtConstraintViolation.
GTEST_TEST(IsBothQuaternionAndQuaternionDtOK, testA) {
  const double half_angle = M_PI/6;
  const double cos_half_angle = std::cos(half_angle);
  const double sin_half_angle = std::sin(half_angle);
  const double x = 0.3, y = 0.7, z = std::sqrt(1.0 - (x*x + y*y));

  // MotionGenesis was used to calculate values for e0Dt, e1Dt, e2Dt, e3Dt that
  // satisifies the constraint  e0*e0' + e1*e1' + e2*e2' + e3*e3' = 0.
  const double e0 = cos_half_angle;
  const double e1 = sin_half_angle * x;
  const double e2 = sin_half_angle * y;
  const double e3 = sin_half_angle * z;
  const double e0Dt = -0.4211981425390414;
  const double e1Dt =  2.387689633338328;
  const double e2Dt = -1.529504299767133;
  const double e3Dt =  1.672467320028763;
  const Eigen::Quaterniond quat(e0,  e1,  e2,  e3);
  const drake::Vector4<double> quatDt(e0Dt, e1Dt, e2Dt, e3Dt);
  const drake::Vector4<double> quatDt_bad(quatDt[0], quatDt[1], quatDt[2],
                                          0.99*quatDt[3]);

  const double epsilon = std::numeric_limits<double>::epsilon();
  const bool a = IsBothQuaternionAndQuaternionDtOK(quat, quatDt,     8*epsilon);
  const bool b = IsBothQuaternionAndQuaternionDtOK(quat, quatDt_bad, 0.01);
  EXPECT_TRUE(a);
  EXPECT_FALSE(b);
}


}  // namespace quaternion_test
}  // namespace math
}  // namespace drake
