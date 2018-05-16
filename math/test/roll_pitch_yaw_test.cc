#include "drake/math/roll_pitch_yaw.h"

#include <cmath>

#include <gtest/gtest.h>

#include "drake/common/drake_assert.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/math/rotation_matrix.h"

namespace drake {
namespace math {
namespace {

const double kEpsilon = std::numeric_limits<double>::epsilon();

// This tests the RollPitchYaw constructors and IsNearlyEqualTo().
GTEST_TEST(RollPitchYaw, testConstructorsAndIsNearlyEqualTo) {
  const RollPitchYaw<double> a(0.1, 0.2, -0.3);
  const RollPitchYaw<double> b(0.1, 0.2, -0.4);
  const RollPitchYaw<double> c(Eigen::Vector3d(0.1, 0.2, -0.3));
  EXPECT_FALSE(a.IsNearlyEqualTo(b, kEpsilon));
  EXPECT_TRUE(a.IsNearlyEqualTo(c, kEpsilon));
  EXPECT_FALSE(a.IsNearlyEqualTo(b, 0.1 - 10*kEpsilon));
  EXPECT_TRUE(a.IsNearlyEqualTo(b, 0.1 + 10*kEpsilon));
}

// This tests the RollPitchYaw access methods.
GTEST_TEST(RollPitchYaw, testAcessMethods) {
  const RollPitchYaw<double> rpy(0.12, 0.34, -0.56);
  const Eigen::Vector3d v = rpy.vector();
  const double roll = rpy.get_roll_angle();
  const double pitch = rpy.get_pitch_angle();
  const double yaw = rpy.get_yaw_angle();
  EXPECT_TRUE(v(0) == roll && roll == 0.12);
  EXPECT_TRUE(v(1) == pitch && pitch == 0.34);
  EXPECT_TRUE(v(2) == yaw && yaw == -0.56);
}

// This tests the RollPitchYaw.ToQuaternion() method.
GTEST_TEST(RollPitchYaw, testToQuaternion) {
  const RollPitchYaw<double> rpy(0.12, 0.34, -0.56);
  const Eigen::Quaterniond quat = rpy.ToQuaternion();
  const RotationMatrix<double> R1(rpy);
  const RotationMatrix<double> R2(quat);
  EXPECT_TRUE(R1.IsNearlyEqualTo(R2, kEpsilon));
}

// This tests the RollPitchYaw.IsValid() method.
GTEST_TEST(RollPitchYaw, testIsValid) {
  const double kInfinity = std::numeric_limits<double>::infinity();
  const Eigen::Vector3d a(0.12, 0.34, -0.56);
  const Eigen::Vector3d b(0, 0, NAN);
  const Eigen::Vector3d c(0, 0, kInfinity);
  EXPECT_TRUE(RollPitchYaw<double>::IsValid(a));
  EXPECT_FALSE(RollPitchYaw<double>::IsValid(b));
  EXPECT_FALSE(RollPitchYaw<double>::IsValid(c));
}

// For a rotation matrix R that depends on roll-pitch-yaw angles `rpy`,
// calculate the ordinary derivative of R with respect to t.
GTEST_TEST(RollPitchYaw, OrdinaryDerivativeRotationMatrixRollPitchYaw) {
  const RollPitchYaw<double> rpy(0.2, 0.3, 0.4);
  const Eigen::Vector3d rpyDt(-2.1, 3.3, 5.7);
  const Eigen::Matrix3d RDt = rpy.OrdinaryDerivativeRotationMatrix(rpyDt);

  // Results generated by MotionGenesis.
  const double r = rpy.get_roll_angle();
  const double p = rpy.get_pitch_angle();
  const double y = rpy.get_yaw_angle();
  const double c0 = cos(r),  c1 = cos(p),  c2 = cos(y);
  const double s0 = sin(r),  s1 = sin(p),  s2 = sin(y);
  const double rDt = rpyDt(0), pDt = rpyDt(1), yDt = rpyDt(2);
  Eigen::Matrix3d MDt;
  MDt << -s1*c2*pDt - s2*c1*yDt,
      s0*s2*rDt + s1*c0*c2*rDt + s0*c1*c2*pDt - c0*c2*yDt - s1*s0*s2*yDt,
      s0*c2*yDt + s2*c0*rDt + c1*c0*c2*pDt - s1*s0*c2*rDt - s1*s2*c0*yDt,
      c1*c2*yDt - s1*s2*pDt,
      s1*s0*c2*yDt + s1*s2*c0*rDt + s0*s2*c1*pDt - s0*c2*rDt - s2*c0*yDt,
      s0*s2*yDt + s1*c0*c2*yDt + s2*c1*c0*pDt - c0*c2*rDt - s1*s0*s2*rDt,
      -c1*pDt,
      c1*c0*rDt - s1*s0*pDt,
      -s1*c0*pDt - s0*c1*rDt;

  EXPECT_TRUE(CompareMatrices(RDt, MDt, 32 * kEpsilon,
                              MatrixCompareType::absolute));
}

// For a RollPitchYaw rpy that relates orientation of a frame A to a frame D,
// calculate conversion from rpy and its time-derivative rpyDt to w_AD_A
// (D's angular velocity in A, expressed in A).
GTEST_TEST(RollPitchYaw, RollPitchYawDtToAngularVelocityExpressedInA) {
  const RollPitchYaw<double> rpy(0.2, 0.3, 0.4);
  const Eigen::Vector3d rpyDt(-2.1, 3.3, 5.7);
  const Eigen::Vector3d w_AD_A = rpy.RollPitchYawDtToAngularVelocityA(rpyDt);

  // Results generated by MotionGenesis.
  const double p = rpy.get_pitch_angle();
  const double y = rpy.get_yaw_angle();
  const double rDt = rpyDt(0), pDt = rpyDt(1), yDt = rpyDt(2);
  using std::cos;
  using std::sin;
  const double wx = cos(p) * cos(y) * rDt - sin(y) *pDt;
  const double wy = cos(y) * pDt + sin(y) * cos(p) * rDt;
  const double wz = yDt - sin(p) * rDt;
  const Eigen::Vector3d w_AD_A_expected(wx, wy, wz);

  EXPECT_TRUE(CompareMatrices(w_AD_A, w_AD_A_expected, 32 * kEpsilon,
                              MatrixCompareType::absolute));
}



}  // namespace
}  // namespace math
}  // namespace drake
