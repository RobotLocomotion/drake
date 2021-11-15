#include "drake/math/rigid_transform.h"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_no_throw.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/math/autodiff.h"

namespace drake {
namespace math {
namespace {

using Eigen::Matrix3d;
using Eigen::Vector3d;
using symbolic::Expression;
using symbolic::Formula;
using symbolic::Variable;

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

  // Bad rotation matrix should throw exception.
  // Note: Although this test seems redundant with similar tests for the
  // RotationMatrix class, it is here due to the bug (mentioned below) in
  // EXPECT_THROW.  Contrast the use here of EXPECT_THROW (which does not have
  // an extra set of parentheses around its first argument) with the use of
  // EXPECT_THROW((RigidTransform<double>(isometryC)), std::logic_error); below.
  const Matrix3d bad = GetBadRotationMatrix();
  if (kDrakeAssertIsArmed) {
    EXPECT_THROW(RigidTransform<double>(RotationMatrix<double>(bad), p),
                 std::logic_error);
  }
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

// Test constructing a RigidTransform from a 3x4 matrix.
GTEST_TEST(RigidTransform, ConstructorFromMatrix34) {
  const RotationMatrixd R = GetRotationMatrixB();
  const Vector3<double> position(4, 5, 6);
  Eigen::Matrix<double, 3, 4> pose;
  pose << R.matrix(), position;
  const RigidTransformd X(pose);
  EXPECT_TRUE(CompareMatrices(X.GetAsMatrix34(), pose));

  if (kDrakeAssertIsArmed) {
    pose(2, 2) += 1E-5;  // Corrupt the last element of the rotation matrix.
    EXPECT_THROW(RigidTransformd XX(pose), std::logic_error);
  }
}

// Test constructing a RigidTransform from a 4x4 matrix.
GTEST_TEST(RigidTransform, ConstructorFromMatrix4) {
  const RotationMatrixd R = GetRotationMatrixB();
  const Vector3<double> position(4, 5, 6);
  Matrix4<double> pose;
  pose << R.matrix(), position, 0, 0, 0, 1;
  const RigidTransformd X(pose);
  EXPECT_TRUE(CompareMatrices(X.GetAsMatrix4(), pose));

  // Ensure the 4x4 constructor fails if the last row differs from [0, 0, 0, 1].
  if (kDrakeAssertIsArmed) {
    DRAKE_EXPECT_NO_THROW(RigidTransformd Xm(pose));
    pose(3, 0) = kEpsilon;
    EXPECT_THROW(RigidTransformd Xm(pose), std::logic_error);
    pose(3, 0) = 0;  pose(3, 1) = kEpsilon;
    EXPECT_THROW(RigidTransformd Xm(pose), std::logic_error);
    pose(3, 1) = 0;  pose(3, 2) = kEpsilon;
    EXPECT_THROW(RigidTransformd Xm(pose), std::logic_error);
    pose(3, 2) = 0;  pose(3, 3) = 1 + 2 * kEpsilon;
    EXPECT_THROW(RigidTransformd Xm(pose), std::logic_error);
  }
}

// Test constructing a RigidTransform from an Eigen expression.
// Valid expressions need to resolve to Vector3, 3x4, or 4x4 matrix.
GTEST_TEST(RigidTransform, ConstructorFromEigenExpression) {
  // Test constructor with a Vector3 Eigen expression.
  const Vector3<double> position(4, 5, 6);
  const RigidTransform<double> X1(3 * position);
  EXPECT_TRUE(X1.rotation().IsExactlyIdentity());
  EXPECT_TRUE(X1.translation() == 3 * position);

  // Test constructor with a 3x4 matrix Eigen expression.
  const RotationMatrix<double> R(RollPitchYaw<double>(1, 2, 3));
  Eigen::Matrix<double, 3, 4> pose34;
  pose34 << R.matrix(), position;
  const RigidTransform<double> X2((1 + kEpsilon) * pose34);
  EXPECT_TRUE(CompareMatrices(X2.GetAsMatrix34(), (1 + kEpsilon) * pose34));

  // Test constructor with a 4x4 matrix Eigen expression.
  Eigen::Matrix<double, 4, 4> pose4;
  pose4 << R.matrix(), position, 0, 0, 0, 1;
  const RigidTransform<double> X3(pose4 * pose4);
  EXPECT_TRUE(CompareMatrices(X3.GetAsMatrix4(), pose4 * pose4));

  // Ensure the 4x4 constructor fails if the last row differs from [0, 0, 0, 1].
  if (kDrakeAssertIsArmed) {
    DRAKE_EXPECT_NO_THROW(RigidTransformd Xm(pose4 * pose4));
    pose4(3, 0) = kEpsilon;
    EXPECT_THROW(RigidTransformd Xm(pose4 * pose4), std::logic_error);
    pose4(3, 0) = 0;  pose4(3, 1) = kEpsilon;
    EXPECT_THROW(RigidTransformd Xm(pose4 * pose4), std::logic_error);
    pose4(3, 1) = 0;  pose4(3, 2) = kEpsilon;
    EXPECT_THROW(RigidTransformd Xm(pose4 * pose4), std::logic_error);
    pose4(3, 2) = 0;  pose4(3, 3) = 1 + 2 * kEpsilon;
    EXPECT_THROW(RigidTransformd Xm(pose4 * pose4), std::logic_error);
  }

  // Ensure calling the constructor with a 3x3 matrix Eigen expression fails.
  if (kDrakeAssertIsArmed) {
    const Matrix3<double> m3 = R.matrix();  // 3x3 matrix.
    EXPECT_THROW(RigidTransformd Xm(1.0 * m3), std::logic_error);
  }
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

  // Bad matrix should throw exception.
  const Matrix3d bad = GetBadRotationMatrix();
  Isometry3<double> isometryC;
  isometryC.linear() = bad;
  // Note: As of December 2017, there seems to be a bug in EXPECT_THROW.
  // The next line incorrectly calls the default RigidTransform constructor.
  // The default RigidTransform constructor does not throw an exception which
  // means the EXPECT_THROW fails.  The fix (credit Sherm) was to add an extra
  // set of parentheses around the first argument of EXPECT_THROW.
  if (kDrakeAssertIsArmed) {
    EXPECT_THROW((RigidTransform<double>(isometryC)), std::logic_error);
  }
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

// Tests RigidTransform multiplied by another RigidTransform, for both the
// infix operator*() and assignment operator*=(). Note that these are
// specialized for double, so we have to test both double and some non-double
// type to make sure both paths are exercised.
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

  // Let's try to create the same result with operator*=().
  RigidTransform<double> will_be_X_CA(X_CB);
  will_be_X_CA *= X_BA;
  EXPECT_TRUE(will_be_X_CA.IsNearlyEqualTo(X_CA_expected, 32 * kEpsilon));

  // Repeat both tests for the non-double implementations.
  const RigidTransform<Expression> X_BAx = X_BA.cast<Expression>();
  const RigidTransform<Expression> X_CBx = X_CB.cast<Expression>();
  const RigidTransform<Expression> X_CAx = X_CBx * X_BAx;
  EXPECT_TRUE(CompareMatrices(symbolic::Evaluate(X_CAx.GetAsMatrix34()),
                              X_CA_expected.GetAsMatrix34(), 32 * kEpsilon));

  RigidTransform<Expression> will_be_X_CAx(X_CBx);
  will_be_X_CAx *= X_BAx;
  EXPECT_TRUE(CompareMatrices(symbolic::Evaluate(will_be_X_CAx.GetAsMatrix34()),
                              X_CA_expected.GetAsMatrix34(), 32 * kEpsilon));
}

// Test the faster combined invert-then-compose method. Like the multiply
// operators, the implementation of InvertAndCompose() is specialized for double
// so we need to test both double and one other scalar type to make sure both
// paths are exercised.
GTEST_TEST(RigidTransform, InvertAndCompose) {
  const RigidTransform<double> X_BA = GetRigidTransformA();
  const RigidTransform<double> X_BC = GetRigidTransformB();

  // The inverse() method and multiply operator are tested separately.
  const RigidTransform<double> X_AC_expected = X_BA.inverse() * X_BC;

  // This is what we're testing here.
  const RigidTransform<double> X_AC = X_BA.InvertAndCompose(X_BC);
  EXPECT_TRUE(X_AC.IsNearlyEqualTo(X_AC_expected, 32 * kEpsilon));

  // Now check the implementation for T ≠ double.
  const RigidTransform<Expression> X_BAx = X_BA.cast<Expression>();
  const RigidTransform<Expression> X_BCx = X_BC.cast<Expression>();
  const RigidTransform<Expression> X_ACx = X_BAx.InvertAndCompose(X_BCx);
  EXPECT_TRUE(CompareMatrices(symbolic::Evaluate(X_ACx.GetAsMatrix34()),
                              X_AC_expected.GetAsMatrix34(), 32 * kEpsilon));
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
// construction and methods involving Bool specialized for Expression, namely:
// IsExactlyIdentity(), IsIdentityToEpsilon(), IsNearlyEqualTo().
GTEST_TEST(RigidTransform, SymbolicRigidTransformSimpleTests) {
  // Test RigidTransform can be constructed with Expression.
  RigidTransform<Expression> X;

  // Test IsExactlyIdentity() nominally works with Expression.
  Formula test_Bool = X.IsExactlyIdentity();
  EXPECT_TRUE(test_Bool);

  // Test IsIdentityToEpsilon() nominally works with Expression.
  test_Bool = X.IsIdentityToEpsilon(kEpsilon);
  EXPECT_TRUE(test_Bool);

  // Test IsExactlyEqualTo() nominally works for Expression.
  const RigidTransform<Expression>& X_built_in_identity =
      RigidTransform<Expression>::Identity();
  test_Bool = X.IsExactlyEqualTo(X_built_in_identity);
  EXPECT_TRUE(test_Bool);

  // Test IsNearlyEqualTo() nominally works for Expression.
  test_Bool = X.IsNearlyEqualTo(X_built_in_identity, kEpsilon);
  EXPECT_TRUE(test_Bool);

  // Now perform the same tests on a non-identity transform.
  const Vector3<Expression> p_symbolic(1, 2, 3);
  X.set_translation(p_symbolic);

  // Test IsExactlyIdentity() works with Expression.
  test_Bool = X.IsExactlyIdentity();
  EXPECT_FALSE(test_Bool);

  // Test IsIdentityToEpsilon() works with Expression.
  test_Bool = X.IsIdentityToEpsilon(kEpsilon);
  EXPECT_FALSE(test_Bool);

  // Test IsExactlyEqualTo() works for Expression.
  test_Bool = X.IsExactlyEqualTo(X_built_in_identity);
  EXPECT_FALSE(test_Bool);

  // Test IsNearlyEqualTo() works for Expression.
  test_Bool = X.IsNearlyEqualTo(X_built_in_identity, kEpsilon);
  EXPECT_FALSE(test_Bool);
}

// Test that symbolic conversions may throw exceptions.
GTEST_TEST(RigidTransform, SymbolicRigidTransformThrowsExceptions) {
  const Variable x("x");  // Arbitrary variable.
  Matrix3<Expression> m_symbolic;
  m_symbolic << 1, 0, 0,
                0, 1, 0,
                0, 0, x;  // Not necessarily an identity matrix.
  RotationMatrix<Expression> R_symbolic(m_symbolic);
  const Vector3<Expression> p_symbolic(0, 0, 0);
  const RigidTransform<Expression> X_symbolic(R_symbolic, p_symbolic);

  // The next four tests should throw exceptions since the tests are
  // inconclusive because the value of x is unknown.
  Formula test_Bool = X_symbolic.IsExactlyIdentity();
  EXPECT_THROW(test_Bool.Evaluate(), std::runtime_error);

  test_Bool = X_symbolic.IsIdentityToEpsilon(kEpsilon);
  EXPECT_THROW(test_Bool.Evaluate(), std::runtime_error);

  const RigidTransform<Expression>& X_identity =
      RigidTransform<Expression>::Identity();
  test_Bool = X_symbolic.IsExactlyEqualTo(X_identity);
  EXPECT_THROW(test_Bool.Evaluate(), std::runtime_error);

  test_Bool = X_symbolic.IsNearlyEqualTo(X_identity, kEpsilon);
  EXPECT_THROW(test_Bool.Evaluate(), std::runtime_error);
}

// Test constructing a RigidTransform constructor from an Eigen::Translation3.
GTEST_TEST(RigidTransform, ConstructRigidTransformFromTranslation3) {
  const Vector3d p_AoBo_A(1.0, 2.0, 3.0);
  const Eigen::Translation3d translation3(p_AoBo_A);
  const RigidTransformd X_AB(translation3);            // Explicit construction
  const RigidTransformd X_AB_implicit = translation3;  // Implicit construction
  EXPECT_EQ(X_AB.translation(), p_AoBo_A);
  EXPECT_TRUE(X_AB.rotation().IsExactlyIdentity());
  EXPECT_TRUE(X_AB.IsExactlyEqualTo(X_AB_implicit));
}

// Test the RigidTransform set_rotation() methods.
GTEST_TEST(RigidTransform, SetRotationMethods) {
  const Vector3d p_AoBo_A(1.0, 2.0, 3.0);
  RigidTransformd X_AB(p_AoBo_A);
  EXPECT_TRUE(X_AB.rotation().IsExactlyIdentity());

  // Test RigidTransform set_rotation() method with a RollPitchYaw.
  const RollPitchYaw<double> rpy(0.1, 0.2, 0.3);
  X_AB.set_rotation(rpy);
  EXPECT_TRUE(X_AB.rotation().IsExactlyEqualTo(RotationMatrix<double>(rpy)));
  EXPECT_EQ(X_AB.translation(), p_AoBo_A);
  X_AB.set_rotation(RotationMatrix<double>::Identity());
  EXPECT_TRUE(X_AB.rotation().IsExactlyIdentity());

  // Test RigidTransform set_rotation() method with a Quaternion.
  Eigen::Quaterniond quat(1, 2, 3, 4);
  quat.normalize();
  X_AB.set_rotation(quat);
  EXPECT_TRUE(X_AB.rotation().IsExactlyEqualTo(RotationMatrix<double>(quat)));
  EXPECT_EQ(X_AB.translation(), p_AoBo_A);
  X_AB.set_rotation(RotationMatrix<double>::Identity());
  EXPECT_TRUE(X_AB.rotation().IsExactlyIdentity());

  // Test RigidTransform set_rotation() method with a Quaternion.
  // Choose a unit vector using a Pythagorean quadruple, i.e., a set of integers
  // a, b, c and d, such that a² + b² + c² = d².  One set is [1, 2, 2, 3].
  const Eigen::Vector3d axis(1.0 / 3.0, 2.0 / 3.0, 2.0 / 3.0);
  const Eigen::AngleAxis<double> angle_axis(0.234, axis);
  X_AB.set_rotation(angle_axis);
  EXPECT_TRUE(
      X_AB.rotation().IsExactlyEqualTo(RotationMatrix<double>(angle_axis)));
  EXPECT_EQ(X_AB.translation(), p_AoBo_A);
  X_AB.set_rotation(RotationMatrix<double>::Identity());
  EXPECT_TRUE(X_AB.rotation().IsExactlyIdentity());
}

// Test multiplying a RigidTransform by an Eigen::Translation3 and vice-versa.
GTEST_TEST(RigidTransform, OperatorMultiplyByTranslation3AndViceVersa) {
  const Vector3d p_AoBo_A(1.0, 2.0, 3.0);
  const RotationMatrixd R_AB(RollPitchYawd(0.3, 0.2, 0.7));
  const RigidTransformd X_AB(R_AB, p_AoBo_A);

  const Vector3d p_BoCo_B(4.0, 5.0, 9.0);
  const Eigen::Translation3d X_BC(p_BoCo_B);

  // Multiply a RigidTransform by a Translation3.
  const RigidTransformd X_AC = X_AB * X_BC;

  // Verify the rotation portion of the previous multiply.
  const RotationMatrixd& R_AC = X_AC.rotation();
  EXPECT_TRUE(R_AC.IsExactlyEqualTo(X_AB.rotation()));

  // Verify the translation portion of the previous multiply.
  const Vector3d p_BoCo_A = R_AB * p_BoCo_B;
  const Vector3d p_AoCo_A = p_AoBo_A + p_BoCo_A;
  EXPECT_TRUE(X_AC.translation().isApprox(p_AoCo_A, 32 * kEpsilon));

  // Test multiplying a Translation3 by a RigidTransform.
  const Eigen::Translation3d X_CB = X_BC.inverse();
  const RigidTransformd X_BA = X_AB.inverse();
  const RigidTransformd X_CA = X_CB * X_BA;

  // Verify the rotation portion of the previous multiply.
  const RotationMatrixd& R_CA = X_CA.rotation();
  const RotationMatrixd& R_BA = X_BA.rotation();
  EXPECT_TRUE(R_CA.IsExactlyEqualTo(R_BA));
  const RotationMatrixd  R_CB = RotationMatrixd::Identity();

  // Verify the translation portion of the previous multiply.
  const Vector3d& p_CoBo_C = X_CB.translation();
  const Vector3d& p_BoAo_B = X_BA.translation();
  const Vector3d p_BoAo_C = R_CB * p_BoAo_B;
  const Vector3d p_CoAo_C = p_CoBo_C + p_BoAo_C;
  EXPECT_TRUE(X_CA.translation().isApprox(p_CoAo_C, 32 * kEpsilon));

  // Verify X_AC is the inverse of X_CA.
  EXPECT_TRUE(X_AC.IsNearlyEqualTo(X_CA.inverse(), 32 * kEpsilon));
}

// Tests RigidTransform X_AB multiplied by a 3 x n matrix whose columns are
// regarded as position vectors from Bo (frame B's origin) to an arbitrary point
// Qi, expressed in B.  The result is tested to be a 3 x n matrix whose columns
// are position vectors from Ao (frame A's origin) to Qi, expressed in A.
GTEST_TEST(RigidTransform, OperatorMultiplyByMatrix3X) {
  // Create a generic RigidTransform having a rotation matrix whose elements are
  // all non-zero elements and a position vector having all non-zero elements.
  const RigidTransform<double> X_AB = GetRigidTransformA();

  // Multiply the RigidTransform X_AB by 3 position vectors to test operator*
  // for a 3 x n matrix, where n = 3 is known before compilation.
  Eigen::Matrix3d p_BoQ_B;
  const Vector3d p_BoQ1_B(-12, -9, 7);   p_BoQ_B.col(0) = p_BoQ1_B;
  const Vector3d p_BoQ2_B(-11, -8, 10);  p_BoQ_B.col(1) = p_BoQ2_B;
  const Vector3d p_BoQ3_B(-10, -7, 12);  p_BoQ_B.col(2) = p_BoQ3_B;
  const auto p_AoQ_A = X_AB * p_BoQ_B;

  // Ensure the compiler's declared type for p_AoQ_A has the proper number of
  // rows and columns before compilation.  Then verify the results.
  EXPECT_EQ(decltype(p_AoQ_A)::RowsAtCompileTime, 3);
  EXPECT_EQ(decltype(p_AoQ_A)::ColsAtCompileTime, 3);
  EXPECT_TRUE(CompareMatrices(p_AoQ_A.col(0), X_AB * p_BoQ1_B, kEpsilon));
  EXPECT_TRUE(CompareMatrices(p_AoQ_A.col(1), X_AB * p_BoQ2_B, kEpsilon));
  EXPECT_TRUE(CompareMatrices(p_AoQ_A.col(2), X_AB * p_BoQ3_B, kEpsilon));

  // Multiply the RigidTransform X_AB by n position vectors to test operator*
  // for a 3 x n matrix, where n is not known before compilation.
  const int number_of_position_vectors = 2;
  Eigen::Matrix3Xd p_BoP_B(3, number_of_position_vectors);
  p_BoP_B.col(0) = p_BoQ1_B;
  p_BoP_B.col(1) = p_BoQ2_B;
  const auto p_AoP_A = X_AB * p_BoP_B;

  // Ensure the compiler's declared type for p_AoP_A has the proper number of
  // rows before compilation (dictated by the return type of operator*) and
  // has the proper number of columns at run time.
  EXPECT_EQ(decltype(p_AoP_A)::RowsAtCompileTime, 3);
  EXPECT_EQ(p_AoP_A.cols(), number_of_position_vectors);
  for (int i = 0; i < number_of_position_vectors; ++i) {
    const Vector3d p_AoPi_A = p_AoP_A.col(i);
    const Vector3d p_AoPi_A_expected = p_AoP_A.col(i);  // Previous result.
    EXPECT_TRUE(CompareMatrices(p_AoPi_A, p_AoPi_A_expected, kEpsilon));
  }

  // Test RigidTransform operator* can multiply an Eigen expression, namely the
  // Eigen expression arising from a 3x1 matrix multiplied by a 1x4 matrix.
  const Eigen::MatrixXd p_AoR_A = X_AB * (Eigen::Vector3d(1, 2, 3) *
                                          Eigen::RowVector4d(1, 2, 3, 4));
  EXPECT_EQ(p_AoR_A.rows(), 3);
  EXPECT_EQ(p_AoR_A.cols(), 4);

  // Test RigidTransform operator* can multiply a different looking Eigen
  // expression that produces the same result.
  const auto p_AoR_A_expected = X_AB *
       (Eigen::MatrixXd(3, 4) << Eigen::Vector3d(1, 2, 3),
                                 Eigen::Vector3d(2, 4, 6),
                                 Eigen::Vector3d(3, 6, 9),
                                 Eigen::Vector3d(4, 8, 12)).finished();
  EXPECT_EQ(decltype(p_AoR_A_expected)::RowsAtCompileTime, 3);
  EXPECT_EQ(p_AoR_A_expected.cols(), 4);
  EXPECT_TRUE(CompareMatrices(p_AoR_A, p_AoR_A_expected, kEpsilon));

  // Test that operator* disallows weirdly-sized matrix multiplication.
  if (kDrakeAssertIsArmed) {
    Eigen::MatrixXd m_7x8(7, 8);
    m_7x8 = Eigen::MatrixXd::Identity(7, 8);
    Eigen::MatrixXd bad_matrix_multiply;
    EXPECT_THROW(bad_matrix_multiply = X_AB * m_7x8, std::logic_error);
  }
}

GTEST_TEST(RigidTransform, TestMemoryLayoutOfRigidTransformDouble) {
  // For optimization (e.g., AVX instructions), verify RigidTransform<double>
  // is packed into 12 consecutive doubles, first with a 3x3 rotation matrix
  // (9 doubles) followed by a 3x1  position vector (3 doubles).
  RigidTransform<double> X;
  const double* X_address = reinterpret_cast<const double*>(&X);
  const double* R_address = reinterpret_cast<const double*>(&X.rotation());
  const double* p_address = reinterpret_cast<const double*>(&X.translation());
  EXPECT_EQ(X_address, R_address);
  EXPECT_EQ(X_address + 9, p_address);

  // Test that the entire class occupies memory equal to 12 doubles.
  EXPECT_EQ(sizeof(X), 12 * sizeof(double));
}

// This utility function helps verify the output string from RigidTransform's
// stream insertion operator <<.  Specifically, it does the following:
// 1. Verifies the output string has form: "rpy = 0.125 0.25 0.5 xyz = 7 6 5";
// 2. Verifies the numerical values for roll (r), pitch (p) and yaw (y) that are
//    contained in the output string are within a 4 epsilon of their expected
//    values, where epsilon ≈ 2.22E-16.
// 3. Verifies that output string's xyz matches (with regular expressions) the
//    expected string.
template <typename T>
void VerifyStreamInsertionOperator(const RigidTransform<T> X_AB,
                                   const Vector3<double>& rpy_expected,
                                   const std::string& xyz_expected_string) {
  // Due to the conversion from a RollPitchYaw to a RotationMatrix and then back
  // to a RollPitchYaw, the input rpy_double may slightly mismatch output, so
  // stream_string may be something like
  // “rpy = 0.12499999999999997 0.25 0.4999999999999999 xyz = 4.0 3.0 2.0
  std::stringstream stream;  stream << X_AB;
  const std::string stream_string = stream.str();
  ASSERT_EQ(stream_string.find("rpy = "), 0);
  const char* cstring = stream_string.c_str() + 6;  // Start of double value.
  double roll, pitch, yaw;
  int match_count = sscanf(cstring, "%lf %lf %lf ", &roll, &pitch, &yaw);
  ASSERT_EQ(match_count, 3) << "When scanning " << stream_string;
  EXPECT_TRUE(CompareMatrices(Vector3<double>(roll, pitch, yaw), rpy_expected,
                              4 * kEpsilon));

  // Verify string contains something like xyz = 7 6 5 or xyz = 7.0 6.0 5.0.
  EXPECT_THAT(stream_string, testing::ContainsRegex(xyz_expected_string));
}

// Test the stream insertion operator to write into a stream.
GTEST_TEST(RigidTransform, StreamInsertionOperator) {
  // Test stream insertion for RigidTransform<double>.
  // Verify streamA.str() is similar to "rpy = 0.125 0.25 0.5 xyz = 4 3 2";
  RollPitchYaw<double> rpy_double(0.125, 0.25, 0.5);
  Vector3<double> xyz_double(4, 3, 2);
  std::string xyz_expected_string = "xyz = 4.* 3.* 2.*";
  VerifyStreamInsertionOperator(RigidTransform<double>(rpy_double, xyz_double),
                                rpy_double.vector(), xyz_expected_string);

  // Test stream insertion for RigidTransform<double> with NaN and inf.
  // Verify streamA.str() is similar to "rpy = 0.125 0.25 0.5 xyz = Inf 3 NaN";
  constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
  constexpr double kInfinity = std::numeric_limits<double>::infinity();
  xyz_double = Vector3<double>(kInfinity, 3, kNaN);
  xyz_expected_string = "xyz = inf 3.* nan";
  VerifyStreamInsertionOperator(RigidTransform<double>(rpy_double, xyz_double),
                                rpy_double.vector(), xyz_expected_string);

  // Test stream insertion for RigidTransform<AutoDiffXd>.
  // Verify streamB.str() is similar to "rpy = -0.33 0.17 0.9 xyz = 7 6 5";
  const RollPitchYaw<AutoDiffXd> rpy_autodiff(-0.33, 0.17, 0.9);
  const Vector3<AutoDiffXd> xyz_autodiff(-17, 987, 6.5432);
  xyz_expected_string = "xyz = -17.* 987.* 6.5432.*";
  const Vector3<double> rpy_values = ExtractValue(rpy_autodiff.vector());
  VerifyStreamInsertionOperator(
      RigidTransform<AutoDiffXd>(rpy_autodiff, xyz_autodiff), rpy_values,
      xyz_expected_string);

  // Test stream insertion for RigidTransform<Expression>.
  // Note: A numerical process calculates RollPitchYaw from a RotationMatrix.
  // Verify that RigidTransform prints a symbolic placeholder for its rotational
  // component (roll-pitch-yaw string) when T = Expression.
  const Variable x("x"), y("y"), z("z");
  const Variable roll("roll"), pitch("pitch"), yaw("yaw");
  const Vector3<Expression> xyz_symbolic(x, y, z);
  RollPitchYaw<Expression> rpy_symbolic(roll, pitch, yaw);
  RigidTransform<Expression> X_symbolic(rpy_symbolic, xyz_symbolic);
  std::stringstream stream;  stream << X_symbolic;
  const std::string expected_string =
      "rpy = <symbolic> <symbolic> <symbolic> xyz = x y z";
  EXPECT_EQ(expected_string, stream.str());

  // Test stream insertion for RigidTransform<Expression> when the expression
  // is only constants (i.e., with no free variables).
  const RollPitchYaw<Expression> rpy_const_expr(-0.1, 0.2, 0.3);
  const Vector3<Expression> xyz_const_expr(-10, 20, 30);
  VerifyStreamInsertionOperator(
      RigidTransform<Expression>(rpy_const_expr, xyz_const_expr),
      Vector3d{-0.1, 0.2, 0.3}, "xyz = -10 20 30");
}

}  // namespace
}  // namespace math
}  // namespace drake
