/// @file
/// Tests that rotation conversion functions are inverses.

#include <cmath>

#include <Eigen/Dense>

#include "gtest/gtest.h"

#include "drake/common/eigen_matrix_compare.h"
#include "drake/math/axis_angle.h"
#include "drake/math/cross_product.h"
#include "drake/math/normalize_vector.h"
#include "drake/math/quaternion.h"
#include "drake/math/roll_pitch_yaw.h"
#include "drake/math/rotation_matrix.h"

using Eigen::Matrix;
using Eigen::Matrix3d;
using Eigen::Quaterniond;
using Eigen::Vector3d;
using Eigen::Vector4d;

namespace drake {
namespace math {
namespace {

void AxisQuatFun(const Vector4d& a) {
  auto q = axis2quat(a);
  // Manually computes one quaternion corresponding to the axis-angle representation.
  Vector3d::PlainObject axis_normalized;
  NormalizeVector(a.head<3>(), axis_normalized);
  Vector4d q_expected;
  q_expected(0) = std::cos(a(3)/2);
  q_expected.tail<3>() = std::sin(a(3)/2) * axis_normalized;

  EXPECT_TRUE(CompareMatrices(q_expected, q, 1E-10, MatrixCompareType::absolute)
  || CompareMatrices(q_expected, -q, 1E-10, MatrixCompareType::absolute));
}

GTEST_TEST(RotationConversionTest, AxisQuat) {
  // First tests the degenerate cases, when the rotation angle is zero.
  AxisQuatFun(Vector4d(1.0, 0.0, 0.0, 0.0));
  AxisQuatFun(Vector4d(0.0, 1.0, 0.0, 0.0));
  AxisQuatFun(Vector4d(0.0, 0.0, 1.0, 0.0));

  // Second tests other degenerate cases, when the rotation angle is 180.
  AxisQuatFun(Vector4d(1.0, 0.0, 0.0, M_PI));
  AxisQuatFun(Vector4d(0.0, 1.0, 0.0, M_PI));
  AxisQuatFun(Vector4d(0.0, 0.0, 1.0, M_PI));

  // Now tests non-degenerate cases.
  AxisQuatFun(Vector4d(0.5, 0.5, sqrt(2)/2, M_PI/3));
  AxisQuatFun(Vector4d(-0.5, -0.5, -sqrt(2)/2, -M_PI/3));
}

void AxisRotmatFun(const Vector4d &a) {
  // Manually computes the rotation matrix from axis-angle representation, using
  // Rodriguez's rotation formula
  // https://en.wikipedia.org/wiki/Rodrigues%27_rotation_formula
  auto axis_skew = VectorToSkewSymmetric(a.head<3>());
  auto rotmat_expected = Matrix3d::Identity() + std::sin(a(3)) *axis_skew
      + (1.0 - std::cos(a(3))) * axis_skew * axis_skew;
  auto rotmat = axis2rotmat(a);
  EXPECT_TRUE(CompareMatrices(rotmat, rotmat_expected, 1E-10, MatrixCompareType::absolute));
}

GTEST_TEST(RotationConversionTest, AxisRotmat) {
  // First tests the degenerate case, corresponding to no rotation.
  AxisRotmatFun(Vector4d(1.0, 0.0, 0.0, 0.0));
  AxisRotmatFun(Vector4d(0.0, 1.0, 0.0, 0.0));
  AxisRotmatFun(Vector4d(0.0, 0.0, 1.0, 0.0));

  // Now tests non-degenerate case.
  AxisRotmatFun(Vector4d(0.5, 0.5, sqrt(2)/2, M_PI/3));
  AxisRotmatFun(Vector4d(-0.5, sqrt(2)/2, 0.5, -M_PI/4));
}

void QuatAxisFun(const Vector4d& quat) {
  auto a = quat2axis(quat);
  EXPECT_NEAR(a.head<3>().norm(), 1, 1E-6);
  auto quat_expected = axis2quat(a);
  EXPECT_TRUE(CompareMatrices(quat_expected, quat, 1E-10, MatrixCompareType::absolute)
  || CompareMatrices(quat_expected, -quat, 1E-10, MatrixCompareType::absolute));
}

GTEST_TEST(RotationConversionTest, QuatAxis) {
  // First tests the degenerate case, corresponding to no rotation.
  QuatAxisFun(Vector4d(1.0, 0.0, 0.0, 0.0));
  QuatAxisFun(Vector4d(-1.0, 0.0, 0.0, 0.0));

  // Second tests another degenerate case, corresponding to 180 degrees of rotation.
  QuatAxisFun(Vector4d(0.0, 1.0, 0.0, 0.0));
  QuatAxisFun(Vector4d(0.0, 0.0, 1.0, 0.0));
  QuatAxisFun(Vector4d(0.0, 0.0, 0.0, 1.0));

  // Tests near-degenerate cases when the angle is close to 0.
  QuatAxisFun(axis2quat(Vector4d(1, 0, 0, 1E-6*M_PI)));
  QuatAxisFun(axis2quat(Vector4d(1, 0, 0, -1E-6*M_PI)));

  // Tests near-degenerate cases when the angle is close to 180
  QuatAxisFun(axis2quat(Vector4d(1, 0, 0, M_PI - 1E-6*M_PI)));
  QuatAxisFun(axis2quat(Vector4d(0.5, 0.5, 1/sqrt(2), M_PI + 1E-6*M_PI)));

  // Now tests non-degenerate case.
  QuatAxisFun(Vector4d(0.5, 0.5, 0.5, 0.5));
  QuatAxisFun(Vector4d(sqrt(2)/2, 0.5, 1/sqrt(6), 1/sqrt(12)));
}


void QuatRotmatFun(const Vector4d& quat) {
  // Computes the axis-angle representation, then converts to rotation matrix
  auto rotmat_expected = axis2rotmat(quat2axis(quat));
  auto rotmat = quat2rotmat(quat);
  EXPECT_TRUE(CompareMatrices(rotmat_expected, rotmat, 1E-10, MatrixCompareType::absolute));
}

GTEST_TEST(RotationConversionTest, QuatRotmat) {
  // First tests the degenerate case, corresponding to no rotation.
  QuatRotmatFun(Vector4d(1.0, 0.0, 0.0, 0.0));
  QuatRotmatFun(Vector4d(-1.0, 0.0, 0.0, 0.0));

  // Now tests non-degenerate cases.
  QuatRotmatFun(Vector4d(0.5, 0.5, 0.5, 0.5));
  QuatRotmatFun(Vector4d(1/sqrt(2), 0.4, 1/sqrt(6), 1/sqrt(12)));
}

void RotmatAxisFun(const Matrix3d &rotmat) {
  // Knowing that axis2rotmat is correct, I will check if the rotmat2axis is the
  // correct inversion matrix.
  auto a = rotmat2axis(rotmat);
  EXPECT_NEAR(a.head<3>().norm(), 1, 1E-6);
  auto rotmat_expected = axis2rotmat(a);
  EXPECT_TRUE(CompareMatrices(rotmat, rotmat_expected, 1E-10, MatrixCompareType::absolute));
  auto a_expected = quat2axis(rotmat2quat(rotmat));
  EXPECT_TRUE(CompareMatrices(a, a_expected, 1E-10, MatrixCompareType::absolute)
    || CompareMatrices(a, -a_expected, 1E-10, MatrixCompareType::absolute));
}

GTEST_TEST(RotationConversionTest, RotmatAxis) {
  // Tests the degenerate case of no rotation.
  Matrix3d R = Matrix3d::Identity();
  RotmatAxisFun(R);

  // Tests the degenerate case of 180 degrees rotation.
  // 180 degrees rotation around z axis
  R.setZero();
  R(0, 0) = -1;
  R(1, 1) = -1;
  R(2, 2) = 1;
  RotmatAxisFun(R);
  // 180 degrees rotation around y axis
  R.setZero();
  R(0, 0) = -1;
  R(1, 1) = 1;
  R(2, 2) = -1;
  RotmatAxisFun(R);
  // 180 degrees rotation around z axis
  R.setZero();
  R(0, 0) = 1;
  R(1, 1) = -1;
  R(2, 2) = -1;
  RotmatAxisFun(R);

  // Tests the case when the angle is very small, close to degeneration
  R = axis2rotmat(Vector4d(0.5*sqrt(2), 0.4*sqrt(2), 0.3*sqrt(2), 1E-6*M_PI));
  RotmatAxisFun(R);
  R = axis2rotmat(Vector4d(0.5*sqrt(2), 0.4*sqrt(2), 0.3*sqrt(2), -1E-6*M_PI));
  RotmatAxisFun(R);

  // Tests the non-degenerate case
  R = axis2rotmat(Vector4d(0.5*sqrt(2), 0.4*sqrt(2), 0.3*sqrt(2), M_PI/6));
  RotmatAxisFun(R);
}
/*class RotationConversionTest : public ::testing::Test {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 protected:
  void SetUp() override {
    q_ << 0.5, 0.5, 0.5, 0.5;
    R_ = quat2rotmat(q_);
  }

  Vector4d q_;  // A quaternion equivalent to R_.
  Matrix3d R_;  // A rotation matrix equivalent to q_.
};

TEST_F(RotationConversionTest, QuatAxis) {
  auto a = quat2axis(q_);
  auto q_back = axis2quat(a);
  EXPECT_NEAR(acos(std::abs(q_.transpose() * q_back)), 0.0, 1e-6);
}

TEST_F(RotationConversionTest, QuatRotmat) {
  Vector4d q_back = rotmat2quat(R_);
  EXPECT_NEAR(acos(std::abs(q_.transpose() * q_back)), 0.0, 1e-6);
}

TEST_F(RotationConversionTest, QuatRPY) {
  Vector3d rpy = quat2rpy(q_);
  Vector4d q_back = rpy2quat(rpy);
  EXPECT_NEAR(acos(std::abs(q_.transpose() * q_back)), 0.0, 1e-6);
}

TEST_F(RotationConversionTest, RotmatAxis) {
  Vector4d a = rotmat2axis(R_);
  Matrix3d R_back = axis2rotmat(a);
  EXPECT_TRUE(CompareMatrices(R_, R_back, 1e-6, MatrixCompareType::absolute));
}

TEST_F(RotationConversionTest, RotmatRPY) {
  Vector3d rpy = rotmat2rpy(R_);
  Matrix3d R_back = rpy2rotmat(rpy);
  EXPECT_TRUE(CompareMatrices(R_, R_back, 1e-6, MatrixCompareType::absolute));
}

TEST_F(RotationConversionTest, AxisRPY) {
  Vector3d rpy = rotmat2rpy(R_);
  Vector4d axis = rpy2axis(rpy);
  Vector3d rpy_back = axis2rpy(axis);
  EXPECT_TRUE(
      CompareMatrices(rpy, rpy_back, 1e-6, MatrixCompareType::absolute));
}

TEST_F(RotationConversionTest, QuatEigenQuaternion) {
  Quaterniond eigenQuat = quat2eigenQuaternion(q_);
  Matrix3d R_expected = quat2rotmat(q_);
  Matrix3d R_eigen = eigenQuat.matrix();
  EXPECT_TRUE(
      CompareMatrices(R_expected, R_eigen, 1e-6, MatrixCompareType::absolute));
}

TEST_F(RotationConversionTest, DRPYRotmat) {
  Vector3d rpy = rotmat2rpy(R_);
  Matrix3d R = rpy2rotmat(rpy);
  Matrix<double, 9, 3> dR = drpy2rotmat(rpy);
  Matrix<double, 9, 3> dR_num = Matrix<double, 9, 3>::Zero();
  for (int i = 0; i < 3; ++i) {
    Vector3d err = Vector3d::Zero();
    err(i) = 1e-7;
    Vector3d rpyi = rpy + err;
    Matrix3d Ri = rpy2rotmat(rpyi);
    Matrix3d Ri_err = (Ri - R) / err(i);
    for (int j = 0; j < 9; j++) {
      dR_num(j, i) = Ri_err(j);
      EXPECT_NEAR(dR(j, i), dR_num(j, i), 1e-3);
    }
  }
}*/

}  // namespace
}  // namespace math
}  // namespace drake
