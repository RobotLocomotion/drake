#include "drake/multibody/multibody_tree/math/rotation_matrix.h"

#include <gtest/gtest.h>

namespace drake {
namespace multibody {
namespace math {
namespace {

using Eigen::Matrix3d;

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

  // Bad matrix should throw exception.
  m << 1, 9000*kEpsilon, 9000*kEpsilon,
       0, cos_theta, sin_theta,
       0, -sin_theta, cos_theta;
  EXPECT_THROW(RotationMatrix<double> R2(m), std::logic_error);
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
  RotationMatrix<double> R = RotationMatrix<double>::MakeIdentity();
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
  RotationMatrix<double> R_inverse = R.Inverse();
  RotationMatrix<double> I = R * R_inverse;
  Matrix3d zero_matrix = Matrix3<double>::Identity() - I.matrix();
  EXPECT_TRUE((zero_matrix.array() == 0).all());
}

// Test access by (i, j) indexes.
GTEST_TEST(RotationMatrix, AccessByIndexes) {
  const double cos_theta = std::cos(0.5);
  const double sin_theta = std::sin(0.5);
  Matrix3d m;
  m << 1, 0, 0,
      0, cos_theta, sin_theta,
      0, -sin_theta, cos_theta;

  RotationMatrix<double> R(m);
  EXPECT_EQ(R(0, 0), m(0, 0));
  EXPECT_EQ(R(0, 1), m(0, 1));
  EXPECT_EQ(R(0, 2), m(0, 2));
  EXPECT_EQ(R(1, 0), m(1, 0));
  EXPECT_EQ(R(1, 1), m(1, 1));
  EXPECT_EQ(R(1, 2), m(1, 2));
  EXPECT_EQ(R(2, 0), m(2, 0));
  EXPECT_EQ(R(2, 1), m(2, 1));
  EXPECT_EQ(R(2, 2), m(2, 2));
}

// Test rotationa matrix multiplication and IsNearlyEqualTo.
GTEST_TEST(RotationMatrix, OperatorMultiplyAndIsNearlyEqualTo) {
  // Create a rotation matrix from a BodyXYZ rotation by angles q1, q2, q3.
  double q1 = 0.2, q2 = 0.3, q3 = 0.4;
  Matrix3d m_BA;
  m_BA << cos(q2)*cos(q3),
          sin(q3)*cos(q1) + sin(q1)*sin(q2)*cos(q3),
          sin(q1)*sin(q3) - sin(q2)*cos(q1)*cos(q3),
         -sin(q3)*cos(q2),
          cos(q1)*cos(q3) - sin(q1)*sin(q2)*sin(q3),
          sin(q1)*cos(q3) + sin(q2)*sin(q3)*cos(q1),
          sin(q2),
         -sin(q1)*cos(q2),
          cos(q1)*cos(q2);

  // Create a rotation matrix from a BodyXYX rotation by angles r1, r2, r3.
  double r1 = 0.5, r2 = 0.5, r3 = 0.7;
  Matrix3d m_CB;
  m_CB << cos(r2),
          sin(r1)*sin(r2),
         -sin(r2)*cos(r1),
          sin(r2)*sin(r3),
          cos(r1)*cos(r3) - sin(r1)*sin(r3)*cos(r2),
          sin(r1)*cos(r3) + sin(r3)*cos(r1)*cos(r2),
          sin(r2)*cos(r3),
         -sin(r3)*cos(r1) - sin(r1)*cos(r2)*cos(r3),
          cos(r1)*cos(r2)*cos(r3) - sin(r1)*sin(r3);

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
}

// Test IsValid.
GTEST_TEST(RotationMatrix, IsValid) {
  const double cos_theta = std::cos(0.5);
  const double sin_theta = std::sin(0.5);
  Matrix3d m;
  m << 1, 0, 0,
       0, cos_theta, sin_theta,
       0, -sin_theta, cos_theta;
  EXPECT_TRUE(RotationMatrix<double>::IsValid(m, 5*kEpsilon));

  m << 1, 10*kEpsilon, 10*kEpsilon,
       0, cos_theta, sin_theta,
       0, -sin_theta, cos_theta;
  EXPECT_FALSE(RotationMatrix<double>::IsValid(m, 5*kEpsilon));
}


}  // namespace
}  // namespace math
}  // namespace multibody
}  // namespace drake
