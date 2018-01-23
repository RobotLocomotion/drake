#include "drake/multibody/multibody_tree/math/rotation_matrix.h"

#include <gtest/gtest.h>

namespace drake {
namespace multibody {
namespace math {
namespace {

using Eigen::Matrix3d;
using Eigen::Vector3d;

constexpr double kEpsilon = std::numeric_limits<double>::epsilon();

// Helper function to create a rotation matrix associated with a BodyXYZ
// rotation by angles q1 = 0.2 radians, q2 = 0.3 radians, q3 = 0.4 radians.
// Note: These matrices must remain BodyXYZ matrices with the specified angles
// q1, q2, q3, as these matrices are used in conjunction with MotionGenesis
// pre-computed solutions based on these exact matrices.
Matrix3d MakeRotationMatrixBodyXYZ() {
  const double q1 = 0.2, q2 = 0.3, q3 = 0.4;
  const double c1 = std::cos(q1), c2 = std::cos(q2), c3 = std::cos(q3);
  const double s1 = std::sin(q1), s2 = std::sin(q2), s3 = std::sin(q3);
  Matrix3d m;
  m << c2 * c3,
      s3 * c1 + s1 * s2 * c3,
      s1 * s3 - s2 * c1 * c3,
      -s3 * c2,
      c1 * c3 - s1 * s2 * s3,
      s1 * c3 + s2 * s3 * c1,
      s2,
      -s1 * c2,
      c1 * c2;
  return m;
}

// Helper function to create a rotation matrix associated with a BodyXYX
// rotation by angles r1 = 0.5 radians, r2 = 0.5 radians, r3 = 0.7 radians.
// Note: These matrices must remain BodyXYX matrices with the specified angles
// r1, r2, r3, as these matrices are used in conjunction with MotionGenesis
// pre-computed solutions based on these exact matrices.
Matrix3d MakeRotationMatrixBodyXYX() {
  const double r1 = 0.5, r2 = 0.5, r3 = 0.7;
  const double c1 = std::cos(r1), c2 = std::cos(r2), c3 = std::cos(r3);
  const double s1 = std::sin(r1), s2 = std::sin(r2), s3 = std::sin(r3);
  Matrix3d m;
  m << c2,
      s1 * s2,
      -s2 * c1,
      s2 * s3,
      c1 * c3 - s1 * s3 * c2,
      s1 * c3 + s3 * c1 * c2,
      s2 * c3,
      -s3 * c1 - s1 * c2 * c3,
      c1 * c2 * c3 - s1 * s3;
  return m;
}

// Test default constructor - should be identity matrix.
GTEST_TEST(RotationMatrix, DefaultRotationMatrixIsIdentity) {
  RotationMatrix<double> R;
  Matrix3d zero_matrix = R.matrix() - Matrix3d::Identity();
  EXPECT_TRUE((zero_matrix.array() == 0).all());
}

// Test constructing a RotationMatrix from a Matrix3.
GTEST_TEST(RotationMatrix, RotationMatrixConstructor) {
  const double cos_theta = std::cos(0.5);
  const double sin_theta = std::sin(0.5);
  Matrix3d m;
  m << 1, 0, 0,
       0, cos_theta, sin_theta,
       0, -sin_theta, cos_theta;

  RotationMatrix<double> R1(m);
  Matrix3d zero_matrix = m - R1.matrix();
  EXPECT_TRUE((zero_matrix.array() == 0).all());

#ifdef DRAKE_ASSERT_IS_ARMED
  // Bad matrix should throw exception.
  m << 1, 9000*kEpsilon, 9000*kEpsilon,
       0, cos_theta, sin_theta,
       0, -sin_theta, cos_theta;
  EXPECT_THROW(RotationMatrix<double> R2(m), std::logic_error);
#endif
}

// Test setting a RotationMatrix from a Matrix3.
GTEST_TEST(RotationMatrix, SetRotationMatrix) {
  const double cos_theta = std::cos(0.5);
  const double sin_theta = std::sin(0.5);
  Matrix3d m;
  m << 1, 0, 0,
      0, cos_theta, sin_theta,
      0, -sin_theta, cos_theta;

  RotationMatrix<double> R;
  R.SetOrThrowIfNotValid(m);
  Matrix3d zero_matrix = m - R.matrix();
  EXPECT_TRUE((zero_matrix.array() == 0).all());

  // Bad matrix should throw exception.
  m << 1, 9000*kEpsilon, 9000*kEpsilon,
      0, cos_theta, sin_theta,
      0, -sin_theta, cos_theta;
  EXPECT_THROW(R.SetOrThrowIfNotValid(m), std::logic_error);
}

// Test setting a RotationMatrix to an identity matrix.
GTEST_TEST(RotationMatrix, MakeIdentityMatrix) {
  const RotationMatrix<double>& R = RotationMatrix<double>::Identity();
  Matrix3d zero_matrix = Matrix3<double>::Identity() - R.matrix();
  EXPECT_TRUE((zero_matrix.array() == 0).all());
}

// Test making a rotation matrix associated with a X-rotation.
GTEST_TEST(RotationMatrix, RotationMatrixX) {
  const double theta = 0.3;
  const Matrix3d m = Eigen::AngleAxisd(theta, Vector3d::UnitX()).matrix();
  RotationMatrix<double> R = RotationMatrix<double>::MakeRotationMatrixX(theta);
  const Matrix3d zero_matrix = m - R.matrix();
  EXPECT_TRUE((zero_matrix.array() == 0).all());
}

// Test making a rotation matrix associated with a Y-rotation.
GTEST_TEST(RotationMatrix, RotationMatrixY) {
  const double theta = 0.4;
  const Matrix3d m = Eigen::AngleAxisd(theta, Vector3d::UnitY()).matrix();
  RotationMatrix<double> R = RotationMatrix<double>::MakeRotationMatrixY(theta);
  const Matrix3d zero_matrix = m - R.matrix();
  EXPECT_TRUE((zero_matrix.array() == 0).all());
}

// Test making a rotation matrix associated with a Z-rotation.
GTEST_TEST(RotationMatrix, RotationMatrixZ) {
  const double theta = 0.5;
  const Matrix3d m = Eigen::AngleAxisd(theta, Vector3d::UnitZ()).matrix();
  RotationMatrix<double> R = RotationMatrix<double>::MakeRotationMatrixZ(theta);
  const Matrix3d zero_matrix = m - R.matrix();
  EXPECT_TRUE((zero_matrix.array() == 0).all());
}

// Test making a rotation matrix associated with a Body-fixed Z-Y-X rotation.
// or with a Space-fixed X-Y-Z rotation.  Also tests method IsExactlyEqualTo().
GTEST_TEST(RotationMatrix, RotationMatrixBodyZYX) {
  const Vector3d q(0.3, 0.4, 0.5);  // yaw-pitch-roll angles.
  const Matrix3d m = (Eigen::AngleAxisd(q(0), Vector3d::UnitZ())
                    * Eigen::AngleAxisd(q(1), Vector3d::UnitY())
                    * Eigen::AngleAxisd(q(2), Vector3d::UnitX())).matrix();
  const RotationMatrix<double> R_eigen(m);
  const RotationMatrix<double> R_bodyZYX =
      RotationMatrix<double>::MakeRotationMatrixBodyZYX(q);
  EXPECT_TRUE(R_bodyZYX.IsNearlyEqualTo(R_eigen, kEpsilon));

  RotationMatrix<double> R1 = RotationMatrix<double>::MakeRotationMatrixZ(q(0));
  RotationMatrix<double> R2 = RotationMatrix<double>::MakeRotationMatrixY(q(1));
  RotationMatrix<double> R3 = RotationMatrix<double>::MakeRotationMatrixX(q(2));
  RotationMatrix<double> R_expected = R1 * R2 * R3;
  EXPECT_TRUE(R_bodyZYX.IsExactlyEqualTo(R_expected));

  // Compare to SpaceXYZ rotation sequence.
  const Vector3d roll_pitch_yaw(q(2), q(1), q(0));
  const RotationMatrix<double> R_spaceXYZ =
      RotationMatrix<double>::MakeRotationMatrixSpaceXYZ(roll_pitch_yaw);
  EXPECT_TRUE(R_spaceXYZ.IsNearlyEqualTo(R_eigen, kEpsilon));
  EXPECT_TRUE(R_spaceXYZ.IsExactlyEqualTo(R_bodyZYX));
}

// Test calculating the inverse of a RotationMatrix.
GTEST_TEST(RotationMatrix, Inverse) {
  const double cos_theta = std::cos(0.5);
  const double sin_theta = std::sin(0.5);
  Matrix3d m;
  m << 1, 0, 0,
      0, cos_theta, sin_theta,
      0, -sin_theta, cos_theta;
  RotationMatrix<double> R(m);
  RotationMatrix<double> RRinv = R * R.inverse();
  const RotationMatrix<double>& I = RotationMatrix<double>::Identity();
  EXPECT_TRUE(RRinv.IsNearlyEqualTo(I, 8 * kEpsilon));
}

// Test rotation matrix multiplication and IsNearlyEqualTo.
GTEST_TEST(RotationMatrix, OperatorMultiplyAndIsNearlyEqualTo) {
  Matrix3d m_BA = MakeRotationMatrixBodyXYZ();
  Matrix3d m_CB = MakeRotationMatrixBodyXYX();

  RotationMatrix<double> R_BA(m_BA);
  RotationMatrix<double> R_CB(m_CB);
  RotationMatrix<double> R_CA = R_CB * R_BA;

  // Expected results (from MotionGenesis).
  Matrix3d m_CA;
  m_CA << 0.5623597514496498, 0.6644746169581934, -0.4921635839512615,
          0.3778794976730916, 0.3228981562377939, 0.8677233810014371,
          0.7354988750418453, -0.6739512327435091, -0.06950640758724619;
  RotationMatrix<double> R_CA_expected(m_CA);

  // Also test IsNearlyEqualTo.
  EXPECT_TRUE(R_CA.IsNearlyEqualTo(R_CA_expected, 10 * kEpsilon));

  // Also test operator*=().
  R_CB *= R_BA;
  EXPECT_TRUE(R_CB.IsNearlyEqualTo(R_CA, 10 * kEpsilon));
  EXPECT_FALSE(R_CB.IsNearlyEqualTo(R_BA, 10000 * kEpsilon));

  // Also test operator*() with vectors.
  Vector3d vA(1, 2, 3);     // Vector v expressed in frame A.
  Vector3d vC = R_CA * vA;  // Vector v expressed in frame C.
  Vector3d vC_expected = m_CA * vA;
  EXPECT_TRUE(vC.isApprox(vC_expected));
}

// Test IsDeterminantPositive, IsOrthonormal, IsValid.
GTEST_TEST(RotationMatrix, IsValid) {
  const double cos_theta = std::cos(0.5);
  const double sin_theta = std::sin(0.5);
  Matrix3d m;
  m << 1, 0, 0,
       0, cos_theta, sin_theta,
       0, -sin_theta, cos_theta;
  EXPECT_TRUE(RotationMatrix<double>::IsDeterminantPositive(m));
  EXPECT_TRUE(RotationMatrix<double>::IsOrthonormal(m, 5 * kEpsilon));
  EXPECT_TRUE(RotationMatrix<double>::IsValid(m, 5 * kEpsilon));

  // Test a matrix that should fail orthonormality check.
  m << 1, 10 * kEpsilon, 10 * kEpsilon,
       0, cos_theta, sin_theta,
       0, -sin_theta, cos_theta;
  EXPECT_TRUE(RotationMatrix<double>::IsDeterminantPositive(m));
  EXPECT_FALSE(RotationMatrix<double>::IsOrthonormal(m, 5 * kEpsilon));
  EXPECT_FALSE(RotationMatrix<double>::IsValid(m, 5 * kEpsilon));

  // Test a matrix that should fail determinant test.
  m << -1, 0, 0,
        0, cos_theta, sin_theta,
        0, -sin_theta, cos_theta;
  EXPECT_FALSE(RotationMatrix<double>::IsDeterminantPositive(m));
  EXPECT_TRUE(RotationMatrix<double>::IsOrthonormal(m, 5 * kEpsilon));
  EXPECT_FALSE(RotationMatrix<double>::IsValid(m, 5 * kEpsilon));
}

// Tests whether or not a RotationMatrix is an identity matrix.
GTEST_TEST(RotationMatrix, IsExactlyIdentity) {
  const double cos_theta = std::cos(0.5);
  const double sin_theta = std::sin(0.5);
  Matrix3d m;
  m << 1, 0, 0,
      0, cos_theta, sin_theta,
      0, -sin_theta, cos_theta;

  const RotationMatrix<double> R1(m);
  const RotationMatrix<double> R2;
  EXPECT_FALSE(R1.IsExactlyIdentity());
  EXPECT_TRUE(R2.IsExactlyIdentity());
}

// Test ProjectMatrixToRotationMatrix.
GTEST_TEST(RotationMatrix, ProjectToRotationMatrix) {
  Matrix3d m;
  m << 1, 0.1, 0.1, -0.2, 1.0, 0.1, 0.5, 0.6, 0.8;
  EXPECT_FALSE(RotationMatrix<double>::IsValid(m, 64000 * kEpsilon));
  RotationMatrix<double> R = RotationMatrix<double>::ProjectToRotationMatrix(m);
  EXPECT_TRUE(R.IsValid());

  m << 1, 2, 3, 4, 5, 6, 7, 8, -10;
  R = RotationMatrix<double>::ProjectToRotationMatrix(m);
  EXPECT_TRUE(R.IsValid());

  m << 1E-7, 2, 3, 4, 5, 6, 7, 8, -1E6;
  R = RotationMatrix<double>::ProjectToRotationMatrix(m);
  EXPECT_TRUE(R.IsValid());

  m << 0, 0, 0, 0, 0, 0, 0, 0, 0;
  R = RotationMatrix<double>::ProjectToRotationMatrix(m);
  EXPECT_TRUE(R.IsValid());

  m << 1, 2, 3, 4, 5, 6, 7, 8, 9;
  EXPECT_FALSE(RotationMatrix<double>::IsDeterminantPositive(m));
  EXPECT_THROW(RotationMatrix<double>::ProjectToRotationMatrix(m),
               std::logic_error);
}

// Test RotationMatrix cast method from double to AutoDiffXd.
GTEST_TEST(RotationMatrix, CastFromDoubleToAutoDiffXd) {
  const Matrix3d m = MakeRotationMatrixBodyXYZ();
  const RotationMatrix<double> R_double(m);
  const RotationMatrix<AutoDiffXd> R_autodiff = R_double.cast<AutoDiffXd>();

  // To avoid a (perhaps) tautological test, do not just use an Eigen cast() to
  // the Matrix3 that underlies the RotationMatrix class -- i.e., avoid just
  // comparing m_autodiff.cast<double>() with m_double.
  // Instead, check element-by-element equality as follows.
  const Matrix3<double>& m_double = R_double.matrix();
  const Matrix3<AutoDiffXd>& m_autodiff = R_autodiff.matrix();
  for (int i = 0;  i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      const double mij_double = m_double(i, j);
      const AutoDiffXd& mij_autodiff = m_autodiff(i, j);
      EXPECT_EQ(mij_autodiff.value(), mij_double);
      EXPECT_EQ(mij_autodiff.derivatives().size(), 0);
    }
  }
}


}  // namespace
}  // namespace math
}  // namespace multibody
}  // namespace drake
