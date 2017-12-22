#include "drake/multibody/multibody_tree/math/rotation_matrix.h"

#include <gtest/gtest.h>

namespace drake {
namespace multibody {
namespace math {
namespace {

using Eigen::Matrix3d;
using Eigen::Vector3d;

constexpr double kEpsilon = std::numeric_limits<double>::epsilon();

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
  // Create a rotation matrix from a BodyXYZ rotation by angles q1, q2, q3.
  double q1 = 0.2, q2 = 0.3, q3 = 0.4;
  double c1 = std::cos(q1), c2 = std::cos(q2), c3 = std::cos(q3);
  double s1 = std::sin(q1), s2 = std::sin(q2), s3 = std::sin(q3);
  Matrix3d m_BA;
  m_BA << c2 * c3,
          s3 * c1 + s1 * s2 * c3,
          s1 * s3 - s2 * c1 * c3,
         -s3 * c2,
          c1 * c3 - s1 * s2 * s3,
          s1 * c3 + s2 * s3 * c1,
          s2,
         -s1 * c2,
          c1 * c2;

  // Create a rotation matrix from a BodyXYX rotation by angles r1, r2, r3.
  double r1 = 0.5, r2 = 0.5, r3 = 0.7;
  c1 = std::cos(r1), c2 = std::cos(r2), c3 = std::cos(r3);
  s1 = std::sin(r1), s2 = std::sin(r2), s3 = std::sin(r3);
  Matrix3d m_CB;
  m_CB << c2,
          s1 * s2,
         -s2 * c1,
          s2 * s3,
          c1 * c3 - s1 * s3 * c2,
          s1 * c3 + s3 * c1 * c2,
          s2 * c3,
         -s3 * c1 - s1 * c2 * c3,
          c1 * c2 * c3 - s1 * s3;

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

}  // namespace
}  // namespace math
}  // namespace multibody
}  // namespace drake
