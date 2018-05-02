#include "drake/math/transform.h"

#include <gtest/gtest.h>

namespace drake {
namespace math {
namespace {

using Eigen::Matrix3d;
using Eigen::Vector3d;

constexpr double kEpsilon = std::numeric_limits<double>::epsilon();

// Helper function to create a rotation matrix associated with a BodyXYZ
// rotation by angles q1, q2, q3.
// Note: These matrices must remain BodyXYZ matrices with the specified angles
// q1, q2, q3, as these matrices are used in conjunction with MotionGenesis
// pre-computed solutions based on these exact matrices.
RotationMatrix<double> GetRotationMatrixA() {
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
  return RotationMatrix<double>(m);
}

// Helper function to create a rotation matrix associated with a BodyXYX
// rotation by angles r1, r2, r3.
// Note: These matrices must remain BodyXYX matrices with the specified angles
// r1, r2, r3, as these matrices are used in conjunction with MotionGenesis
// pre-computed solutions based on these exact matrices.
RotationMatrix<double> GetRotationMatrixB() {
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
  return RotationMatrix<double>(m);
}

#ifdef DRAKE_ASSERT_IS_ARMED
// Helper function to create an invalid rotation matrix.
Matrix3d GetBadRotationMatrix() {
  const double theta = 0.5;
  const double cos_theta = std::cos(theta), sin_theta = std::sin(theta);
  Matrix3d m;
  m << 1, 9000*kEpsilon, 9000*kEpsilon,
       0, cos_theta, sin_theta,
       0, -sin_theta, cos_theta;
  return m;
}
#endif

// Helper functions to create generic position vectors.
Vector3d GetPositionVectorA() { return Vector3d(2, 3, 4); }
Vector3d GetPositionVectorB() { return Vector3d(5, 6, 7); }

// Helper function to create generic transforms.
Transform<double> GetTransformA() {
  return Transform<double>(GetRotationMatrixA(), GetPositionVectorA());
}

Transform<double> GetTransformB() {
  return Transform<double>(GetRotationMatrixB(), GetPositionVectorB());
}

// Tests default constructor - should be identity transform.
GTEST_TEST(Transform, DefaultTransformIsIdentity) {
  const Transform<double> X;
  EXPECT_TRUE(X.IsExactlyIdentity());
}

// Tests constructing a Transform from a RotationMatrix and Vector3.
GTEST_TEST(Transform, TransformConstructor) {
  const RotationMatrix<double> R1 = GetRotationMatrixB();
  const Matrix3d m = R1.matrix();
  const Vector3<double> p(4, 5, 6);
  const Transform<double> X(R1, p);
  const Matrix3d zero_rotation = m - X.rotation().matrix();
  const Vector3d zero_position = p - X.translation();
  EXPECT_TRUE((zero_rotation.array() == 0).all());
  EXPECT_TRUE((zero_position.array() == 0).all());

#ifdef DRAKE_ASSERT_IS_ARMED
  // Bad rotation matrix should throw exception.
  // Note: Although this test seems redundant with similar tests for the
  // RotationMatrix class, it is here due to the bug (mentioned below) in
  // EXPECT_THROW.  Contrast the use here of EXPECT_THROW (which does not have
  // an extra set of parentheses around its first argument) with the use of
  // EXPECT_THROW((Transform<double>(isometryC)), std::logic_error); below.
  const Matrix3d bad = GetBadRotationMatrix();
  EXPECT_THROW(Transform<double>(RotationMatrix<double>(bad), p),
               std::logic_error);
#endif
}

// Tests getting a 4x4 matrix from a Transform.
GTEST_TEST(Transform, Matrix44) {
  const RotationMatrix<double> R = GetRotationMatrixB();
  const Vector3<double> p(4, 5, 6);
  const Transform<double> X(R, p);
  const Matrix4<double> Y = X.GetAsMatrix4();
  const Matrix3d m = R.matrix();

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
  const RotationMatrix<double> R = GetRotationMatrixB();
  const Matrix3d m = R.matrix();
  const Vector3<double> p(4, 5, 6);
  Isometry3<double> isometryA;
  isometryA.linear() = m;
  isometryA.translation() = p;

  const Transform<double> X(isometryA);
  Matrix3d zero_rotation = m - X.rotation().matrix();
  Vector3d zero_position = p - X.translation();
  EXPECT_TRUE((zero_rotation.array() == 0).all());
  EXPECT_TRUE((zero_position.array() == 0).all());

  // Tests making an Isometry3 from a Transform.
  const Isometry3<double> isometryB = X.GetAsIsometry3();
  zero_rotation = m - isometryB.linear();
  zero_position = p - isometryB.translation();
  EXPECT_TRUE((zero_rotation.array() == 0).all());
  EXPECT_TRUE((zero_position.array() == 0).all());

#ifdef DRAKE_ASSERT_IS_ARMED
  // Bad matrix should throw exception.
  const Matrix3d bad = GetBadRotationMatrix();
  Isometry3<double> isometryC;
  isometryC.linear() = bad;
  // Note: As of December 2017, there seems to be a bug in EXPECT_THROW.
  // The next line incorrectly calls the default Transform constructor.
  // The default Transform constructor does not throw an exception which means
  // the EXPECT_THROW fails.  The fix (credit Sherm) was to add an extra set
  // set of parentheses around the first argument of EXPECT_THROW.
  EXPECT_THROW((Transform<double>(isometryC)), std::logic_error);
#endif
}

// Tests method Identity (identity rotation matrix and zero vector).
GTEST_TEST(Transform, Identity) {
  const Transform<double>& X = Transform<double>::Identity();
  EXPECT_TRUE(X.IsExactlyIdentity());
}

// Tests method SetIdentity.
GTEST_TEST(Transform, SetIdentity) {
  const RotationMatrix<double> R = GetRotationMatrixA();
  const Vector3d p(2, 3, 4);
  Transform<double> X(R, p);
  X.SetIdentity();
  EXPECT_TRUE(X.IsExactlyIdentity());
}

// Tests whether or not a Transform is an identity transform.
GTEST_TEST(Transform, IsIdentity) {
  // Test whether it is an identity matrix multiple ways.
  Transform<double> X1;
  EXPECT_TRUE(X1.IsExactlyIdentity());
  EXPECT_TRUE(X1.IsIdentityToEpsilon(0.0));
  EXPECT_TRUE(X1.rotation().IsExactlyIdentity());
  EXPECT_TRUE((X1.translation().array() == 0).all());

  // Test non-identity matrix.
  const RotationMatrix<double> R = GetRotationMatrixA();
  const Vector3d p(2, 3, 4);
  Transform<double> X2(R, p);
  EXPECT_FALSE(X2.IsExactlyIdentity());

  // Change rotation matrix to identity, but leave non-zero position vector.
  X2.set_rotation(RotationMatrix<double>::Identity());
  EXPECT_FALSE(X2.IsExactlyIdentity());
  EXPECT_FALSE(X2.IsIdentityToEpsilon(3.99));
  EXPECT_TRUE(X2.IsIdentityToEpsilon(4.01));

  // Change position vector to zero vector.
  const Vector3d zero_vector(0, 0, 0);
  X2.set_translation(zero_vector);
  EXPECT_TRUE(X2.IsExactlyIdentity());
}

// Tests calculating the inverse of a Transform.
GTEST_TEST(Transform, Inverse) {
  const RotationMatrix<double> R_AB = GetRotationMatrixA();
  const Vector3d p_AoBo_A(2, 3, 4);
  const Transform<double> X(R_AB, p_AoBo_A);
  const Transform<double> I = X * X.inverse();
  const Transform<double> X_identity = Transform<double>::Identity();
  // As documented in IsNearlyEqualTo(), 8 * epsilon was chosen because it is
  // slightly larger than the characteristic length |p_AoBo_A| = 5.4
  // Note: The square-root of the condition number for a Transform is roughly
  // the magnitude of the position vector.  The accuracy of the calculation for
  // the inverse of a Transform drops off with the sqrt condition number.
  EXPECT_TRUE(I.IsNearlyEqualTo(X_identity, 8 * kEpsilon));
}

// Tests Transform multiplied by another Transform
GTEST_TEST(Transform, OperatorMultiplyByTransform) {
  const Transform<double> X_BA = GetTransformA();
  const Transform<double> X_CB = GetTransformB();
  const Transform<double> X_CA = X_CB * X_BA;

  // Check accuracy of rotation calculations.
  const RotationMatrix<double> R_CA = X_CA.rotation();
  const RotationMatrix<double> R_BA = GetRotationMatrixA();
  const RotationMatrix<double> R_CB = GetRotationMatrixB();
  const RotationMatrix<double> R_CA_expected = R_CB * R_BA;
  EXPECT_TRUE(R_CA.IsNearlyEqualTo(R_CA_expected, 0));

  // Expected position vector (from MotionGenesis).
  const double x_expected = 5.761769695362743;
  const double y_expected = 11.26952907288644;
  const double z_expected = 6.192677089863299;
  const Vector3d p_CoAo_C_expected(x_expected, y_expected, z_expected);

  // Check accuracy of translation calculations.
  const Vector3d p_CoAo_C_actual = X_CA.translation();
  EXPECT_TRUE(p_CoAo_C_actual.isApprox(p_CoAo_C_expected, 32 * kEpsilon));

  // Expected transform (with position vector from MotionGenesis).
  // As documented in IsNearlyEqualTo(), 32 * epsilon was chosen because it is
  // slightly larger than the characteristic length |p_CoAo_C| = 14.2
  const Transform<double> X_CA_expected(R_CA_expected, p_CoAo_C_expected);
  EXPECT_TRUE(X_CA.IsNearlyEqualTo(X_CA_expected, 32 * kEpsilon));
}

// Tests Transform multiplied by a position vector.
GTEST_TEST(Transform, OperatorMultiplyByPositionVector) {
  const Transform<double> X_CB = GetTransformB();

  // Calculate position vector from Co to Q, expressed in C.
  const Vector3d p_BoQ_B = GetPositionVectorA();
  const Vector3d p_CoQ_C = X_CB * p_BoQ_B;

  // Expected position vector (from MotionGenesis).
  const double x_expected = 5.761769695362743;
  const double y_expected = 11.26952907288644;
  const double z_expected = 6.192677089863299;
  const Vector3d p_CoQ_C_expected(x_expected, y_expected, z_expected);

  // Check accuracy of translation calculations.
  EXPECT_TRUE(p_CoQ_C.isApprox(p_CoQ_C_expected, 32 * kEpsilon));
}

// Test Transform cast method from double to AutoDiffXd.
GTEST_TEST(Transform, CastFromDoubleToAutoDiffXd) {
  const RollPitchYaw<double> rpy(0.4, 0.3, 0.2);
  const RotationMatrix<double> R_double(rpy);
  const Vector3d p_double(-5, 3, 9);
  const Transform<double> T_double(R_double, p_double);
  const Transform<AutoDiffXd> T_autodiff = T_double.cast<AutoDiffXd>();

  // To avoid a (perhaps) tautological test, check element-by-element equality.
  const Matrix4<double>& m_double = T_double.GetAsMatrix4();
  const Matrix4<AutoDiffXd>& m_autodiff = T_autodiff.GetAsMatrix4();
  for (int i = 0;  i < 4; i++) {
    for (int j = 0; j < 4; j++) {
      const double mij_double = m_double(i, j);
      const AutoDiffXd& mij_autodiff = m_autodiff(i, j);
      EXPECT_EQ(mij_autodiff.value(), mij_double);
      EXPECT_EQ(mij_autodiff.derivatives().size(), 0);
    }
  }
}

}  // namespace
}  // namespace math
}  // namespace drake
