#include "drake/math/rigid_transform.h"

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

// Helper function to create a generic RigidTransform.
RigidTransform<double> GetRigidTransformA() {
  return RigidTransform<double>(GetRotationMatrixA(), GetPositionVectorA());
}

RigidTransform<double> GetRigidTransformB() {
  return RigidTransform<double>(GetRotationMatrixB(), GetPositionVectorB());
}

// Tests default constructor - should be identity RigidTransform.
GTEST_TEST(RigidTransform, DefaultRigidTransformIsIdentity) {
  const RigidTransform<double> X;
  EXPECT_TRUE(X.IsExactlyIdentity());
}

// Tests constructing a RigidTransform from a RotationMatrix and Vector3.
// Also test the set method.
GTEST_TEST(RigidTransform, ConstructorAndSet) {
  const RotationMatrix<double> R1 = GetRotationMatrixB();
  const Matrix3d m = R1.matrix();
  const Vector3<double> p(4, 5, 6);
  RigidTransform<double> X(R1, p);
  Matrix3d zero_rotation = m - X.rotation().matrix();
  Vector3d zero_position = p - X.translation();
  EXPECT_TRUE((zero_rotation.array() == 0).all());
  EXPECT_TRUE((zero_position.array() == 0).all());

  // Test the set method.
  X.set(RotationMatrix<double>::Identity(), Vector3d(0, 0, 0));
  EXPECT_TRUE(X.IsExactlyIdentity());
  X.set(R1, p);
  zero_rotation = m - X.rotation().matrix();
  zero_position = p - X.translation();
  EXPECT_TRUE((zero_rotation.array() == 0).all());
  EXPECT_TRUE((zero_position.array() == 0).all());

#ifdef DRAKE_ASSERT_IS_ARMED
  // Bad rotation matrix should throw exception.
  // Note: Although this test seems redundant with similar tests for the
  // RotationMatrix class, it is here due to the bug (mentioned below) in
  // EXPECT_THROW.  Contrast the use here of EXPECT_THROW (which does not have
  // an extra set of parentheses around its first argument) with the use of
  // EXPECT_THROW((RigidTransform<double>(isometryC)), std::logic_error); below.
  const Matrix3d bad = GetBadRotationMatrix();
  EXPECT_THROW(RigidTransform<double>(RotationMatrix<double>(bad), p),
               std::logic_error);
#endif
}

// Tests constructing a RigidTransform from just a RotationMatrix.
// Also test alias/typdef RigidTransformd.
GTEST_TEST(RigidTransform, ConstructorFromRotationMatrix) {
  const RotationMatrixd R = GetRotationMatrixB();
  const RigidTransformd X(R);
  EXPECT_TRUE(X.rotation().IsExactlyEqualTo(R));
  EXPECT_TRUE(X.translation() == Vector3d(0, 0, 0));
}

// Tests constructing a RigidTransform from just a position vector.
GTEST_TEST(RigidTransform, ConstructorFromPositionVector) {
  const Vector3<double> p(4, 5, 6);
  const RigidTransform<double> X(p);
  EXPECT_TRUE(X.rotation().IsExactlyIdentity());
  EXPECT_TRUE(X.translation() == p);
}

// Tests constructing a RigidTransform from RollPitchYaw and a position vector.
GTEST_TEST(RigidTransform, ConstructorRollPitchYawPositionVector) {
  const RollPitchYaw<double> rpy(1, 2, 3);
  const Vector3<double> position(4, 5, 6);
  const RigidTransform<double> X(rpy, position);
  // The following test is not a RigidTransform verification.
  // It just tests that the rotation matrix in the transform is the same as the
  // rotation matrix generated directly by the given RollPitchYaw rpy, and that
  // the position vector in the transform is the same as the given position.
  EXPECT_TRUE(X.rotation().IsExactlyEqualTo(rpy.ToRotationMatrix()));
  EXPECT_TRUE(X.translation() == position);
}

// Tests constructing a RigidTransform from a Quaterion and a position vector.
GTEST_TEST(RigidTransform, ConstructorQuaternionPositionVector) {
  // Note: Constructor says that the quaternion does not need to be unit length.
  const Eigen::Quaternion<double> quaternion(1, 2, 3, 4);
  const Vector3<double> position(4, 5, 6);
  const RigidTransform<double> X(quaternion, position);
  // The following test is not a RigidTransform verification.
  // It just tests that the rotation matrix in the transform is the same as the
  // rotation matrix generated directly by the given quaternion, and that
  // the position vector in the transform is the same as the given position.
  const RotationMatrix<double> R(quaternion);
  EXPECT_TRUE(X.rotation().IsExactlyEqualTo(R));
  EXPECT_TRUE(X.translation() == position);
}

// Tests constructing a RigidTransform from an AngleAxis and a position vector.
GTEST_TEST(RigidTransform, ConstructorAngleAxisPositionVector) {
  // Note: Constructor says that the axis does not need to be unit length.
  const Eigen::AngleAxis<double> theta_lambda(0.3, Eigen::Vector3d(3, 2, 1));
  const Vector3<double> position(4, 5, 6);
  const RigidTransform<double> X(theta_lambda, position);
  // The following test is not a RigidTransform verification.
  // It just tests that the rotation matrix in the transform is the same as the
  // rotation matrix generated directly by the given AngleAxis, and that
  // the position vector in the transform is the same as the given position.
  const RotationMatrix<double> R(theta_lambda);
  EXPECT_TRUE(X.rotation().IsExactlyEqualTo(R));
  EXPECT_TRUE(X.translation() == position);
}

// Tests getting a 4x4 and 3x4 matrix from a RigidTransform.
GTEST_TEST(RigidTransform, GetAsMatrices) {
  const RotationMatrix<double> R = GetRotationMatrixB();
  const Matrix3d m = R.matrix();
  const Vector3<double> p(4, 5, 6);
  const RigidTransform<double> X(R, p);

  // Test GetAsMatrix4()
  const Matrix4<double> Y = X.GetAsMatrix4();
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

  // Test GetAsMatrix34()
  const Eigen::Matrix<double, 3, 4> Z = X.GetAsMatrix34();
  EXPECT_EQ(Z(0, 0), m(0, 0));
  EXPECT_EQ(Z(0, 1), m(0, 1));
  EXPECT_EQ(Z(0, 2), m(0, 2));
  EXPECT_EQ(Z(0, 3), p(0));
  EXPECT_EQ(Z(1, 0), m(1, 0));
  EXPECT_EQ(Z(1, 1), m(1, 1));
  EXPECT_EQ(Z(1, 2), m(1, 2));
  EXPECT_EQ(Z(1, 3), p(1));
  EXPECT_EQ(Z(2, 0), m(2, 0));
  EXPECT_EQ(Z(2, 1), m(2, 1));
  EXPECT_EQ(Z(2, 2), m(2, 2));
  EXPECT_EQ(Z(2, 3), p(2));
}

// Tests set/get a RigidTransform with an Isometry3.
GTEST_TEST(RigidTransform, Isometry3) {
  const RotationMatrix<double> R = GetRotationMatrixB();
  const Matrix3d m = R.matrix();
  const Vector3<double> p(4, 5, 6);
  Isometry3<double> isometryA;
  isometryA.linear() = m;
  isometryA.translation() = p;

  // Test constructing a RigidTransform from an Isometry3.
  const RigidTransform<double> X(isometryA);
  Matrix3d zero_rotation = m - X.rotation().matrix();
  Vector3d zero_position = p - X.translation();
  EXPECT_TRUE((zero_rotation.array() == 0).all());
  EXPECT_TRUE((zero_position.array() == 0).all());

  // Tests making an Isometry3 from a RigidTransform.
  // Note: Ensure the last row is 0, 0, 0, 1.
  const Isometry3<double> isometryB = X.GetAsIsometry3();
  zero_rotation = m - isometryB.linear();
  zero_position = p - isometryB.translation();
  EXPECT_TRUE((zero_rotation.array() == 0).all());
  EXPECT_TRUE((zero_position.array() == 0).all());
  EXPECT_EQ(isometryB(3, 0), 0);
  EXPECT_EQ(isometryB(3, 1), 0);
  EXPECT_EQ(isometryB(3, 2), 0);
  EXPECT_EQ(isometryB(3, 3), 1);

  // Test setting a RigidTransform from an Isometry3.
  RigidTransform<double> X2;
  X2.SetFromIsometry3(isometryA);
  zero_rotation = m - X2.rotation().matrix();
  zero_position = p - X2.translation();
  EXPECT_TRUE((zero_rotation.array() == 0).all());
  EXPECT_TRUE((zero_position.array() == 0).all());

#ifdef DRAKE_ASSERT_IS_ARMED
  // Bad matrix should throw exception.
  const Matrix3d bad = GetBadRotationMatrix();
  Isometry3<double> isometryC;
  isometryC.linear() = bad;
  // Note: As of December 2017, there seems to be a bug in EXPECT_THROW.
  // The next line incorrectly calls the default RigidTransform constructor.
  // The default RigidTransform constructor does not throw an exception which
  // means the EXPECT_THROW fails.  The fix (credit Sherm) was to add an extra
  // set of parentheses around the first argument of EXPECT_THROW.
  EXPECT_THROW((RigidTransform<double>(isometryC)), std::logic_error);
#endif
}

// Tests method Identity (identity rotation matrix and zero vector).
GTEST_TEST(RigidTransform, Identity) {
  const RigidTransform<double>& X = RigidTransform<double>::Identity();
  EXPECT_TRUE(X.IsExactlyIdentity());
}

// Tests method SetIdentity.
GTEST_TEST(RigidTransform, SetIdentity) {
  const RotationMatrix<double> R = GetRotationMatrixA();
  const Vector3d p(2, 3, 4);
  RigidTransform<double> X(R, p);
  X.SetIdentity();
  EXPECT_TRUE(X.IsExactlyIdentity());
}

// Tests whether or not a RigidTransform is an identity RigidTransform.
GTEST_TEST(RigidTransform, IsIdentity) {
  // Test whether it is an identity matrix multiple ways.
  RigidTransform<double> X1;
  EXPECT_TRUE(X1.IsExactlyIdentity());
  EXPECT_TRUE(X1.IsIdentityToEpsilon(0.0));
  EXPECT_TRUE(X1.rotation().IsExactlyIdentity());
  EXPECT_TRUE((X1.translation().array() == 0).all());

  // Test non-identity matrix.
  const RotationMatrix<double> R = GetRotationMatrixA();
  const Vector3d p(2, 3, 4);
  RigidTransform<double> X2(R, p);
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

// Tests calculating the inverse of a RigidTransform.
GTEST_TEST(RigidTransform, Inverse) {
  const RotationMatrix<double> R_AB = GetRotationMatrixA();
  const Vector3d p_AoBo_A(2, 3, 4);
  const RigidTransform<double> X(R_AB, p_AoBo_A);
  const RigidTransform<double> I = X * X.inverse();
  const RigidTransform<double> X_identity = RigidTransform<double>::Identity();
  // As documented in IsNearlyEqualTo(), 8 * epsilon was chosen because it is
  // slightly larger than the characteristic length |p_AoBo_A| = 5.4
  // Note: The square-root of a RigidTransform's condition number is roughly the
  // magnitude of the position vector.  The accuracy of the calculation for the
  // inverse of a RigidTransform drops off with sqrt(condition number).
  EXPECT_TRUE(I.IsNearlyEqualTo(X_identity, 8 * kEpsilon));
}

// Tests RigidTransform multiplied by another RigidTransform
GTEST_TEST(RigidTransform, OperatorMultiplyByRigidTransform) {
  const RigidTransform<double> X_BA = GetRigidTransformA();
  const RigidTransform<double> X_CB = GetRigidTransformB();
  const RigidTransform<double> X_CA = X_CB * X_BA;

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

  // Expected RigidTransform (with position vector from MotionGenesis).
  // As documented in IsNearlyEqualTo(), 32 * epsilon was chosen because it is
  // slightly larger than the characteristic length |p_CoAo_C| = 14.2
  const RigidTransform<double> X_CA_expected(R_CA_expected, p_CoAo_C_expected);
  EXPECT_TRUE(X_CA.IsNearlyEqualTo(X_CA_expected, 32 * kEpsilon));
}

// Tests RigidTransform multiplied by a position vector.
GTEST_TEST(RigidTransform, OperatorMultiplyByPositionVector) {
  const RigidTransform<double> X_CB = GetRigidTransformB();

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

// Test RigidTransform cast method from double to AutoDiffXd.
GTEST_TEST(RigidTransform, CastFromDoubleToAutoDiffXd) {
  const RollPitchYaw<double> rpy(0.4, 0.3, 0.2);
  const RotationMatrix<double> R_double(rpy);
  const Vector3d p_double(-5, 3, 9);
  const RigidTransform<double> T_double(R_double, p_double);
  const RigidTransform<AutoDiffXd> T_autodiff = T_double.cast<AutoDiffXd>();

  // To avoid a (perhaps) tautological test, check element-by-element equality.
  const Matrix4<double>& m_double = T_double.GetAsMatrix4();
  const Matrix4<AutoDiffXd>& m_autodiff = T_autodiff.GetAsMatrix4();
  for (int i = 0;  i < 4; i++) {
    for (int j = 0; j < 4; j++) {
      const double mij_double = m_double(i, j);
      const AutoDiffXd& mij_autodiff = m_autodiff(i, j);
      EXPECT_EQ(mij_autodiff, mij_double);
      EXPECT_EQ(mij_autodiff.derivatives().size(), 0);
    }
  }
}

// Verify RigidTransform is compatible with symbolic::Expression. This includes,
// construction and methods involving Bool specialized for symbolic::Expression,
// namely: IsExactlyIdentity(), IsIdentityToEpsilon(), IsNearlyEqualTo().
GTEST_TEST(RigidTransform, SymbolicRigidTransformSimpleTests) {
  // Test RigidTransform can be constructed with symbolic::Expression.
  RigidTransform<symbolic::Expression> X;

  // Test IsExactlyIdentity() nominally works with symbolic::Expression.
  symbolic::Formula test_Bool = X.IsExactlyIdentity();
  EXPECT_TRUE(test_Bool);

  // Test IsIdentityToEpsilon() nominally works with symbolic::Expression.
  test_Bool = X.IsIdentityToEpsilon(kEpsilon);
  EXPECT_TRUE(test_Bool);

  // Test IsExactlyEqualTo() nominally works for symbolic::Expression.
  const RigidTransform<symbolic::Expression>& X_built_in_identity =
      RigidTransform<symbolic::Expression>::Identity();
  test_Bool = X.IsExactlyEqualTo(X_built_in_identity);
  EXPECT_TRUE(test_Bool);

  // Test IsNearlyEqualTo() nominally works for symbolic::Expression.
  test_Bool = X.IsNearlyEqualTo(X_built_in_identity, kEpsilon);
  EXPECT_TRUE(test_Bool);

  // Now perform the same tests on a non-identity transform.
  const Vector3<symbolic::Expression> p_symbolic(1, 2, 3);
  X.set_translation(p_symbolic);

  // Test IsExactlyIdentity() works with symbolic::Expression.
  test_Bool = X.IsExactlyIdentity();
  EXPECT_FALSE(test_Bool);

  // Test IsIdentityToEpsilon() works with symbolic::Expression.
  test_Bool = X.IsIdentityToEpsilon(kEpsilon);
  EXPECT_FALSE(test_Bool);

  // Test IsExactlyEqualTo() works for symbolic::Expression.
  test_Bool = X.IsExactlyEqualTo(X_built_in_identity);
  EXPECT_FALSE(test_Bool);

  // Test IsNearlyEqualTo() works for symbolic::Expression.
  test_Bool = X.IsNearlyEqualTo(X_built_in_identity, kEpsilon);
  EXPECT_FALSE(test_Bool);
}

// Test that symbolic conversions may throw exceptions.
GTEST_TEST(RigidTransform, SymbolicRigidTransformThrowsExceptions) {
  const symbolic::Variable x("x");  // Arbitrary variable.
  Matrix3<symbolic::Expression> m_symbolic;
  m_symbolic << 1, 0, 0,
                0, 1, 0,
                0, 0, x;  // Not necessarily an identity matrix.
  RotationMatrix<symbolic::Expression> R_symbolic(m_symbolic);
  const Vector3<symbolic::Expression> p_symbolic(0, 0, 0);
  const RigidTransform<symbolic::Expression> X_symbolic(R_symbolic, p_symbolic);

  // The next four tests should throw exceptions since the tests are
  // inconclusive because the value of x is unknown.
  symbolic::Formula test_Bool = X_symbolic.IsExactlyIdentity();
  EXPECT_THROW(test_Bool.Evaluate(), std::runtime_error);

  test_Bool = X_symbolic.IsIdentityToEpsilon(kEpsilon);
  EXPECT_THROW(test_Bool.Evaluate(), std::runtime_error);

  const RigidTransform<symbolic::Expression>& X_identity =
      RigidTransform<symbolic::Expression>::Identity();
  test_Bool = X_symbolic.IsExactlyEqualTo(X_identity);
  EXPECT_THROW(test_Bool.Evaluate(), std::runtime_error);

  test_Bool = X_symbolic.IsNearlyEqualTo(X_identity, kEpsilon);
  EXPECT_THROW(test_Bool.Evaluate(), std::runtime_error);
}

}  // namespace
}  // namespace math
}  // namespace drake
