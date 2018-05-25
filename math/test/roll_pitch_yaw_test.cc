#include "drake/math/roll_pitch_yaw.h"

#include <cmath>

#include <gtest/gtest.h>

#include "drake/common/drake_assert.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/math/rotation_matrix.h"

namespace drake {
namespace math {
namespace {

using Eigen::Vector3d;
using Eigen::Matrix3d;

const double kEpsilon = std::numeric_limits<double>::epsilon();

// This tests the RollPitchYaw constructors and IsNearlyEqualTo().
GTEST_TEST(RollPitchYaw, testConstructorsAndIsNearlyEqualTo) {
  const RollPitchYaw<double> a(0.1, 0.2, -0.3);
  const RollPitchYaw<double> b(0.1, 0.2, -0.4);
  const RollPitchYaw<double> c(Vector3d(0.1, 0.2, -0.3));
  EXPECT_FALSE(a.IsNearlyEqualTo(b, kEpsilon));
  EXPECT_TRUE(a.IsNearlyEqualTo(c, kEpsilon));
  EXPECT_FALSE(a.IsNearlyEqualTo(b, 0.1 - 10*kEpsilon));
  EXPECT_TRUE(a.IsNearlyEqualTo(b, 0.1 + 10*kEpsilon));
}

// This tests the RollPitchYaw access methods.
GTEST_TEST(RollPitchYaw, testAcessMethods) {
  const RollPitchYaw<double> rpy(0.12, 0.34, -0.56);
  const Vector3d v = rpy.vector();
  const double roll = rpy.roll_angle();
  const double pitch = rpy.pitch_angle();
  const double yaw = rpy.yaw_angle();
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
  const Vector3d a(0.12, 0.34, -0.56);
  const Vector3d b(0, 0, NAN);
  const Vector3d c(0, 0, kInfinity);
  EXPECT_TRUE(RollPitchYaw<double>::IsValid(a));
  EXPECT_FALSE(RollPitchYaw<double>::IsValid(b));
  EXPECT_FALSE(RollPitchYaw<double>::IsValid(c));
}

// For a rotation matrix R that depends on roll-pitch-yaw angles `rpy`,
// calculate the ordinary derivative of R with respect to t.
GTEST_TEST(RollPitchYaw, OrdinaryDerivativeRotationMatrixRollPitchYaw) {
  const RollPitchYaw<double> rpy(0.2, 0.3, 0.4);
  const Vector3d rpyDt(-2.1, 3.3, 5.7);
  const Matrix3d RDt = rpy.CalcRotationMatrixDt(rpyDt);

  // Results generated by MotionGenesis.
  const double r = rpy.roll_angle();
  const double p = rpy.pitch_angle();
  const double y = rpy.yaw_angle();
  const double c0 = cos(r),  c1 = cos(p),  c2 = cos(y);
  const double s0 = sin(r),  s1 = sin(p),  s2 = sin(y);
  const double rDt = rpyDt(0), pDt = rpyDt(1), yDt = rpyDt(2);
  Matrix3d MDt;
  MDt << -s1*c2*pDt - s2*c1*yDt,
      s0*s2*rDt + s1*c0*c2*rDt + s0*c1*c2*pDt - c0*c2*yDt - s1*s0*s2*yDt,
      s0*c2*yDt + s2*c0*rDt + c1*c0*c2*pDt - s1*s0*c2*rDt - s1*s2*c0*yDt,
      c1*c2*yDt - s1*s2*pDt,
      s1*s0*c2*yDt + s1*s2*c0*rDt + s0*s2*c1*pDt - s0*c2*rDt - s2*c0*yDt,
      s0*s2*yDt + s1*c0*c2*yDt + s2*c1*c0*pDt - c0*c2*rDt - s1*s0*s2*rDt,
      -c1*pDt,
      c1*c0*rDt - s1*s0*pDt,
      -s1*c0*pDt - s0*c1*rDt;

  EXPECT_TRUE(CompareMatrices(RDt, MDt, 16 * kEpsilon,
                              MatrixCompareType::absolute));
}

// For a RollPitchYaw R_AD(rpy) that relates frame D's orientation to frame A,
// calculate conversion from rpy and its time-derivative rpyDt to w_AD_A
// (D's angular velocity in A, expressed in A).
GTEST_TEST(RollPitchYaw, CalcAngularVelocityFromRpyDtAndViceVersa) {
  const RollPitchYaw<double> rpy(0.2, 0.3, 0.4);
  const Vector3d rpyDt(-2.1, 3.3, 5.7);
  const Vector3d w_AD_A = rpy.CalcAngularVelocityInParentFromRpyDt(rpyDt);

  // Results generated by MotionGenesis.
  const double p = rpy.pitch_angle();
  const double y = rpy.yaw_angle();
  const double rDt = rpyDt(0), pDt = rpyDt(1), yDt = rpyDt(2);
  using std::cos;
  using std::sin;
  const double wx = cos(p) * cos(y) * rDt - sin(y) *pDt;
  const double wy = cos(y) * pDt + sin(y) * cos(p) * rDt;
  const double wz = yDt - sin(p) * rDt;
  const Vector3d w_AD_A_expected(wx, wy, wz);

  EXPECT_TRUE(CompareMatrices(w_AD_A, w_AD_A_expected, 16 * kEpsilon,
                              MatrixCompareType::absolute));

  // Now test the inverse relationship.
  const Vector3d rpyDt_calculated =
      rpy.CalcRpyDtFromAngularVelocityInParent(w_AD_A);
  EXPECT_TRUE(CompareMatrices(rpyDt_calculated, rpyDt, 16 * kEpsilon,
                              MatrixCompareType::absolute));

  // Check for some throw conditions.
  const RollPitchYaw<double> rpyA(0.2, M_PI / 2, 0.4);
  EXPECT_THROW(rpyA.CalcRpyDtFromAngularVelocityInParent(w_AD_A),
               std::logic_error);

  const RollPitchYaw<double> rpyB(0.2, -M_PI / 2, 0.4);
  EXPECT_THROW(rpyB.CalcRpyDtFromAngularVelocityInParent(w_AD_A),
               std::logic_error);

  const RollPitchYaw<double> rpyC(0.2, 3 * M_PI / 2, 0.4);
  EXPECT_THROW(rpyC.CalcRpyDtFromAngularVelocityInParent(w_AD_A),
               std::logic_error);

  const RollPitchYaw<double> rpyD(0.2, -3 * M_PI / 2, 0.4);
  EXPECT_THROW(rpyD.CalcRpyDtFromAngularVelocityInParent(w_AD_A),
               std::logic_error);

  const RollPitchYaw<double> rpyE(0.2, 3 * M_PI / 2 + 1E-8, 0.4);
  EXPECT_THROW(rpyE.CalcRpyDtFromAngularVelocityInParent(w_AD_A),
               std::logic_error);

  const RollPitchYaw<double> rpyF(0.2, -3 * M_PI / 2 + 1E-8, 0.4);
  EXPECT_THROW(rpyF.CalcRpyDtFromAngularVelocityInParent(w_AD_A),
               std::logic_error);
}


// For a RollPitchYaw rpy that relates orientation of a frame A to a frame D,
// calculate conversion from rpy and its time-derivative rpyDt to w_AD_D
// (D's angular velocity in A, expressed in D).
GTEST_TEST(RollPitchYaw, CalcAngularVelocityInChildFromRpyDt) {
  const RollPitchYaw<double> rpy(0.2, 0.3, 0.4);
  const Vector3d rpyDt(-2.1, 3.3, 5.7);
  const Vector3d w_AD_D = rpy.CalcAngularVelocityInChildFromRpyDt(rpyDt);

  // Results generated by MotionGenesis.
  const double r = rpy.roll_angle();
  const double p = rpy.pitch_angle();
  const double rDt = rpyDt(0), pDt = rpyDt(1), yDt = rpyDt(2);
  using std::cos;
  using std::sin;
  const double wx = rDt - sin(p) * yDt;
  const double wy = cos(r)*pDt + sin(r) * cos(p) *yDt;
  const double wz = cos(p) * cos(r) * yDt - sin(r) * pDt;
  const Vector3d w_AD_D_expected(wx, wy, wz);

  EXPECT_TRUE(CompareMatrices(w_AD_D, w_AD_D_expected, 16 * kEpsilon,
                              MatrixCompareType::absolute));
}

// For a RollPitchYaw rpy that relates orientation of a frame A to a frame D,
// calculate conversion from alpha_AD_A (D's angular acceleration in A,
// expressed in A) to rpyDDt (2nd time-derivative of rpy), i.e., `[r̈, p̈, ÿ]`.
// Cross-validate with `[r̈, p̈, ÿ]` calculated from alpha_AD_D (D's angular
// acceleration in A, expressed in D).
GTEST_TEST(RollPitchYaw, CalcRpyDDtFromAngularAccel) {
  const Vector3d rpyDt(-2.1, 3.3, 5.7);
  const Vector3d alpha_AD_A(0.5, 0.7, 0.9);

  // Set up to test a reasonable range of values.
  const double tol = 64 * kEpsilon * M_PI;
  const double deg = M_PI / 180;
  for (double roll = -M_PI; roll <= M_PI + tol; roll += 10 * deg) {
    for (double pitch = -M_PI/2; pitch <= M_PI/2 + tol; pitch += 2 * deg) {
      for (double yaw = -M_PI; yaw <= M_PI + tol; yaw += 10 * deg) {
        const RollPitchYaw<double> rpy(roll, pitch, yaw);

        // Calculate [r̈, p̈, ÿ] from alpha_AD_A which is
        // D's angular acceleration in A, expressed in A.
        const double abs_cos_pitch = std::abs(std::cos(pitch));
        const bool is_near_singular = abs_cos_pitch <= 1.0E-5;
        Vector3d rpyDDt;
        if (is_near_singular) {
          EXPECT_THROW(rpyDDt = rpy.CalcRpyDDtFromAngularAccelInParent(
                           rpyDt, alpha_AD_A), std::logic_error);
        } else {
          rpyDDt = rpy.CalcRpyDDtFromAngularAccelInParent(rpyDt, alpha_AD_A);
        }

        // Calculate [r̈, p̈, ÿ] from alpha_AD_D which is
        // D's angular acceleration in A, expressed in D.
        const RotationMatrix<double> R_AD(rpy);
        const Vector3d alpha_AD_D = R_AD.inverse() * alpha_AD_A;
        Vector3d rpyDDt_verify;
        if (is_near_singular) {
          EXPECT_THROW(rpyDDt_verify = rpy.CalcRpyDDtFromAngularAccelInChild(
                           rpyDt, alpha_AD_D), std::logic_error);
        } else {
          rpyDDt_verify =
              rpy.CalcRpyDDtFromAngularAccelInChild(rpyDt, alpha_AD_D);
        }

        if (is_near_singular) continue;

        // These two calculations should produce identical answers.
        const double rpyDDt_norm = rpyDDt.norm();
        const double reasonable_value = rpyDDt_norm > 1E-5 ? rpyDDt_norm : 1E-5;
        const double tolerance = 16 * kEpsilon * reasonable_value;
        EXPECT_TRUE(CompareMatrices(rpyDDt, rpyDDt_verify, tolerance,
                                    MatrixCompareType::absolute));

        // Cross-validate with results generated by MotionGenesis.
        using std::cos;
        using std::sin;
        const double p = rpy.pitch_angle();
        const double y = rpy.yaw_angle();
        const double rDt = rpyDt(0), pDt = rpyDt(1), yDt = rpyDt(2);
        const double alfAx = alpha_AD_A(0);
        const double alfAy = alpha_AD_A(1);
        const double alfAz = alpha_AD_A(2);
        double rDDt = (alfAx * cos(y) + alfAy * sin(y)
                    + pDt * yDt + sin(p) * pDt * rDt) / cos(p);
        double pDDt =  alfAy * cos(y) - alfAx * sin(y) - cos(p) * rDt * yDt;
        double yDDt = alfAz + cos(p) * pDt * rDt + tan(p) *
             (alfAx * cos(y) + alfAy * sin(y) + pDt * yDt + sin(p) * pDt * rDt);
        const Vector3d rpyDDt_expected(rDDt, pDDt, yDDt);
        EXPECT_TRUE(CompareMatrices(rpyDDt, rpyDDt_expected, tolerance,
                              MatrixCompareType::absolute));
      }
    }
  }
}

}  // namespace
}  // namespace math
}  // namespace drake
