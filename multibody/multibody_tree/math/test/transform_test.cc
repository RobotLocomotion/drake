#include "drake/multibody/multibody_tree/math/transform.h"

#include <gtest/gtest.h>

namespace drake {
namespace multibody {
namespace math {
namespace {

using Eigen::Matrix3d;
using Eigen::Vector3d;

constexpr double kEpsilon = std::numeric_limits<double>::epsilon();

// Tests default constructor - should be identity transform.
GTEST_TEST(Transform, DefaultTransformIsIdentity) {
  const Transform<double> X;
  const RotationMatrix<double>& R = X.rotation();
  const Vector3<double>& p = X.translation();
  const Matrix3d zero_matrix = R.matrix() - Matrix3d::Identity();
  EXPECT_TRUE((zero_matrix.array() == 0).all());
  EXPECT_TRUE((p.array() == 0).all());
}

// Tests constructing a Transform from a RotationMatrix and Vector3.
GTEST_TEST(Transform, TransformConstructor) {
  const double cos_theta = std::cos(0.5);
  const double sin_theta = std::sin(0.5);
  Matrix3d m;
  m << 1, 0, 0,
       0, cos_theta, sin_theta,
       0, -sin_theta, cos_theta;

  const RotationMatrix<double> R1(m);
  const Vector3<double> p(4, 5, 6);
  const Transform<double> X(R1, p);
  const Matrix3d zero_rotation = m - X.rotation().matrix();
  const Vector3d zero_position = p - X.translation();
  EXPECT_TRUE((zero_rotation.array() == 0).all());
  EXPECT_TRUE((zero_position.array() == 0).all());

#ifdef DRAKE_ASSERT_IS_ARMED
  // Bad matrix should throw exception.
  m << 1, 9000*kEpsilon, 9000*kEpsilon,
       0, cos_theta, sin_theta,
       0, -sin_theta, cos_theta;
  EXPECT_THROW(Transform(RotationMatrix<double> R2(m), p), std::logic_error);
#endif
}

// Tests getting a 4x4 matrix from a Transform.
GTEST_TEST(Transform, Matrix44) {
  const double cos_theta = std::cos(0.5);
  const double sin_theta = std::sin(0.5);
  Matrix3d m;
  m << 1, 0, 0,
       0, cos_theta, sin_theta,
       0, -sin_theta, cos_theta;
  const RotationMatrix<double> R(m);
  const Vector3<double> p(4, 5, 6);
  const Transform<double> X(R, p);
  const Matrix4<double> Y = X.GetAsMatrix();

  EXPECT_EQ(Y(0, 0), m(0, 0));
  EXPECT_EQ(Y(0, 1), m(0, 1));
  EXPECT_EQ(Y(0, 2), m(0, 2));
  EXPECT_EQ(Y(0, 3), p(0));
  EXPECT_EQ(Y(1, 0), m(1, 0));
  EXPECT_EQ(Y(1, 1), m(1, 1));
  EXPECT_EQ(Y(1, 2), m(1, 2));
  EXPECT_EQ(Y(1, 3), p(1));
  EXPECT_EQ(Y(2, 0), m(2, 0));
  EXPECT_EQ(Y(2, 1), m(2, 1));
  EXPECT_EQ(Y(2, 2), m(2, 2));
  EXPECT_EQ(Y(2, 3), p(2));
  EXPECT_EQ(Y(3, 0), 0);
  EXPECT_EQ(Y(3, 1), 0);
  EXPECT_EQ(Y(3, 2), 0);
  EXPECT_EQ(Y(3, 3), 1);
}

// Tests set/get a Transform with an Isometry3.
GTEST_TEST(Transform, Isometry3) {
  const double cos_theta = std::cos(0.5);
  const double sin_theta = std::sin(0.5);
  Matrix3d m;
  m << 1, 0, 0,
       0, cos_theta, sin_theta,
       0, -sin_theta, cos_theta;

  const Vector3<double> p(4, 5, 6);
  Isometry3<double> isometry3;
  isometry3.linear() = m;
  isometry3.translation() = p;

  const Transform<double> X(isometry3);
  Matrix3d zero_rotation = m - X.rotation().matrix();
  Vector3d zero_position = p - X.translation();
  EXPECT_TRUE((zero_rotation.array() == 0).all());
  EXPECT_TRUE((zero_position.array() == 0).all());

  // Tests making an Isometry3 from a RotationMatrix.
  isometry3 = X.GetAsIsometry3();
  zero_rotation = m - isometry3.linear();
  zero_position = p - isometry3.translation();
  EXPECT_TRUE((zero_rotation.array() == 0).all());
  EXPECT_TRUE((zero_position.array() == 0).all());

#ifdef DRAKE_ASSERT_IS_ARMED
  // Bad matrix should throw exception.
  m << 1, 9000*kEpsilon, 9000*kEpsilon,
       0, cos_theta, sin_theta,
       0, -sin_theta, cos_theta;
  isometry3.linear() = m;
  EXPECT_THROW(Transform(isometry3), std::logic_error);
#endif
}

// Tests method MakeIdentity (identity rotation matrix and zero vector).
GTEST_TEST(Transform, MakeIdentity) {
  const Transform<double> X = Transform<double>::MakeIdentity();
  const RotationMatrix<double> R = RotationMatrix<double>::MakeIdentity();
  EXPECT_TRUE(X.rotation().IsNearlyEqualTo(R, 8 * kEpsilon));
  EXPECT_TRUE((X.translation().array() == 0).all());
}

// Tests method SetIdentityMatrixAndZeroPositionVector.
GTEST_TEST(Transform, SetIdentityMatrixAndZeroPositionVector) {
  const RotationMatrix<double> R = RotationMatrix<double>::MakeIdentity();
  const Vector3d p(2, 3, 4);
  Transform<double> X(R, p);
  X.SetIdentityMatrixAndZeroPositionVector();
  EXPECT_TRUE(X.rotation().IsNearlyEqualTo(R, 8 * kEpsilon));
  EXPECT_TRUE((X.translation().array() == 0).all());
}

// Tests calculating the inverse of a Transform.
GTEST_TEST(Transform, Inverse) {
  const double cos_theta = std::cos(0.5);
  const double sin_theta = std::sin(0.5);
  Matrix3d m;
  m << 1, 0, 0,
       0, cos_theta, sin_theta,
       0, -sin_theta, cos_theta;
  const RotationMatrix<double> R_AB(m);
  const Vector3d p_AoBo_A(2, 3, 4);
  const Transform<double> X(R_AB, p_AoBo_A);
  const Transform<double> I = X * X.inverse();
  const Transform<double> X_identity = Transform<double>::MakeIdentity();
  EXPECT_TRUE(I.IsNearlyEqualTo(X_identity, 8 * kEpsilon));
}

// Tests Transform multiplied by another Transform or Vector3.
GTEST_TEST(Transform, OperatorMultiply) {
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
  const RotationMatrix<double> R_BA(m_BA);

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
  const RotationMatrix<double> R_CB(m_CB);

  // Create a position vector from Bo to Ao, expressed in A.
  const Vector3d p_BoAo_B(2, 3, 4);

  // Create a position vector from Co to Bo, expressed in C.
  const Vector3d p_CoBo_C(5, 6, 7);

  // Create associated transforms.
  Transform<double> X_BA(R_BA, p_BoAo_B);
  Transform<double> X_CB(R_CB, p_CoBo_C);
  Transform<double> X_CA = X_CB * X_BA;
  const RotationMatrix<double> R_CA = X_CA.rotation();
  EXPECT_TRUE(R_CA.IsNearlyEqualTo(R_CB * R_BA, 8 * kEpsilon));

  // Expected position vector (from MotionGenesis).
  const double x_expected = 5.761769695362743;
  const double y_expected = 11.26952907288644;
  const double z_expected = 6.192677089863299;
  const Vector3d p_Co_Ao_C_expected(x_expected, y_expected, z_expected);
  const Vector3d p_Co_Ao_C_actual = X_CA.translation();
  EXPECT_TRUE(p_Co_Ao_C_actual.isApprox(p_Co_Ao_C_expected, 16 * kEpsilon));

  // Tests multiplying a Transform by a position vector.
  const Vector3d p_Co_Ao_C = X_CB * p_BoAo_B;
  EXPECT_TRUE(p_Co_Ao_C.isApprox(p_Co_Ao_C_expected, 16 * kEpsilon));

  // Tests GetMaximumAbsoluteDifference.
  double max_diff = X_CB.GetMaximumAbsoluteDifference(X_BA);
  EXPECT_TRUE(max_diff > 2.99 && max_diff < 3.01);

  // Tests GetMaximumAbsoluteTranslationDifference.
  max_diff = X_CB.GetMaximumAbsoluteTranslationDifference(X_BA);
  EXPECT_TRUE(max_diff > 2.99 && max_diff < 3.01);

  // EXPECT_TRUE(R_CB.IsNearlyEqualTo(R_CA, 10 * kEpsilon));
  // EXPECT_FALSE(R_CB.IsNearlyEqualTo(R_BA, 10000 * kEpsilon));
}

}  // namespace
}  // namespace math
}  // namespace multibody
}  // namespace drake
