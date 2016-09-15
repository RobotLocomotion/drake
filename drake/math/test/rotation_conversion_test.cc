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
using std::sin;
using std::cos;

namespace drake {
namespace math {
namespace {
class RotationConversionTest : public ::testing::Test {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 protected:
  void SetUp() override {
    initializeAngleAxisTestCases();
    initializeQuaternionTestCases();
    initializeRotationMatrixTestCases();
  }

  void initializeAngleAxisTestCases() {
    // Tests conversion from axis-angle, tests 10 cases:
    // 1. Degenerate case, no rotation around x axis.
    // 2. Degenerate case, no rotation around y axis.
    // 3. Degenerate case, no rotation around z axis.
    // 4. Degenerate case, 180 degrees rotation around x axis.
    // 5. Degenerate case, 180 degrees rotation around y axis.
    // 6. Degenerate case, 180 degrees rotation around z axis.
    // 7. Almost degenerate case, small positive rotation angle.
    // 8. Almost degenerate case, small negative rotation angle.
    // 9. Almost degenerate case, close to 180 rotation angle.
    // 10. Non-degenerate case.

    // Case 1. no rotation around x axis.
    a_[0] << 1, 0, 0, 0;
    // Case 2. no rotation around y axis.
    a_[1] << 0, 1, 0, 0;
    // Case 3. no rotation around z axis.
    a_[2] << 0, 0, 1, 0;
    // Case 4. 180 degrees rotation around x axis.
    a_[3] << 1, 0, 0, M_PI;
    // Case 5, 180 degrees rotation around y axis.
    a_[4] << 0, 1, 0, M_PI;
    // Case 6, 180 degrees rotation around z axis.
    a_[5] << 0, 0, 1, M_PI;
    // Case 7, small positive rotation angle
    a_[6] << 0.5*sqrt(2), 0.4*sqrt(2), 0.3*sqrt(2), 1E-6*M_PI;
    // Case 8, small negative rotation angle
    a_[7] << 0.5*sqrt(2), 0.4*sqrt(2), 0.3*sqrt(2), -1E-6*M_PI;
    // Case 9, close to 180 rotation angle
    a_[8] << 0.5*sqrt(2), 0.4*sqrt(2), 0.3*sqrt(2), (1-1E-6)*M_PI;
    // Case 10, non-degenerate case
    a_[9] << 0.5*sqrt(2), 0.4*sqrt(2), 0.3*sqrt(2), M_PI/4;
  }

  void initializeQuaternionTestCases() {
    // Tests conversion from quaternion, tests 9 cases:
    // 1. Degenerate case, no rotation.
    // 2. Degenerate case, no rotation.
    // 3. Degenerate case, 180 rotation around x axis.
    // 4. Degenerate case, 180 rotation around y axis.
    // 5. Degenerate case, 180 rotation around z axis.
    // 6. Almost degenerate case, small positive rotation angle.
    // 7. Almost degenerate case, small negative rotation angle.
    // 8. Almost degenerate case, close to 180 rotation.
    // 9. Non-degenerate case.

    // Case 1. no rotation.
    q_[0] << 1, 0, 0, 0;
    // Case 2. no rotation.
    q_[1] << -1, 0, 0, 0;
    // Case 3. 180 rotation around x axis.
    q_[2] << 0, 1, 0, 0;
    // Case 4. 180 rotation around y axis.
    q_[3] << 0, 0, 1, 0;
    // Case 5. 180 rotation around z axis.
    q_[4] << 0, 0, 0, 1;
    // Case 6. Almost degenerate case, small positive rotation angle.
    q_[5] << cos(1E-6*M_PI), 0.5*sqrt(2)*sin(1E-6*M_PI), 0.4*sqrt(2)*sin(1E-6*M_PI), 0.3*sqrt(2)*sin(1E-6*M_PI);
    // Case 7. Almost degenerate case, small negative rotation angle.
    q_[6] << cos(-1E-6*M_PI), 0.5*sqrt(2)*sin(-1E-6*M_PI), 0.4*sqrt(2)*sin(-1E-6*M_PI), 0.3*sqrt(2)*sin(-1E-6*M_PI);
    // Case 8. Almost degenerate case, close to 180 degree rotation.
    q_[7] << cos((1-1E-6)*M_PI), 0.5*sqrt(2)*sin((1-1E-6)*M_PI), 0.4*sqrt(2)*sin((1-1E-6)*M_PI), 0.3*sqrt(2)*sin((1-1E-6)*M_PI);
    // Case 9. Non-degenerate case.
    q_[8] << 1/sqrt(2), 0.5, 1/sqrt(6), 1/sqrt(12);
  }

  void initializeRotationMatrixTestCases() {
    // Tests conversion from rotation matrix, tests 7 cases:
    // 1. Degenerate case, no rotation
    // 2. Degenerate case, 180 degrees rotation around z axis
    // 3. Degenerate case, 180 degrees rotation around y axis
    // 4. Degenerate case, 180 degrees rotation around x axis
    // 5. Almost degenerate case, very small positive rotation angle
    // 6. Almost degenerate case, very small negative rotation angle
    // 7. Almost degenerate case, close 180 degree rotation.
    // 7. Non-degenerage case

    // Case 1. no rotation
    R_[0] = Matrix3d::Identity();

    // Case 2. 180 degrees rotation around z axis
    R_[1] = Matrix3d::Zero();
    R_[1](0, 0) = -1;
    R_[1](1, 1) = -1;
    R_[1](2, 2) = 1;

    // Case 3. 180 degrees rotation around y axis
    R_[2] = Matrix3d::Zero();
    R_[2](0, 0) = -1;
    R_[2](1, 1) = 1;
    R_[2](2, 2) = -1;

    // Case 4. 180 degrees rotation around x axis
    R_[3] = Matrix3d::Zero();
    R_[3](0, 0) = 1;
    R_[3](1, 1) = -1;
    R_[3](2, 2) = -1;

    // Case 5. small positive rotation angle
    R_[4] = axis2rotmat(Vector4d(0.5*sqrt(2), 0.4*sqrt(2), 0.3*sqrt(2), 1E-6*M_PI));

    // Case 6. small negative rotation angle
    R_[5] = axis2rotmat(Vector4d(0.5*sqrt(2), 0.4*sqrt(2), 0.3*sqrt(2), -1E-6*M_PI));

    // Case 7. non-degenerate case
    R_[6] = axis2rotmat(Vector4d(0.5*sqrt(2), 0.4*sqrt(2), 0.3*sqrt(2), M_PI/6));
  }

  void initializeRollPitchYawTestCases() {
    // Tests conversion from roll pitch yaw angles. Tests cases:
    // 1. no rotation
    // 2. Degenerate case, pitch = pi/2;
    // 3. Non-degenerate case.

    // Case 1, no rotation
    rpy_[0] <<0, 0, 0;
    // Case 2, pitch = pi/2;
    rpy_[1]
  };
  static const int kNumAxisAngleTestCases = 10;
  static const int kNumQuaternionTestCases = 9;
  static const int kNumRotationMatrixTestCases = 7;
  static const int kNumRollPitchYawTestCases;
  Vector4d a_[kNumAxisAngleTestCases]; // axis-angle representations to be tested
  Vector4d q_[kNumQuaternionTestCases];  // quaternions to be tested
  Matrix3d R_[kNumRotationMatrixTestCases];  // rotation matrices to be tested
  Vector3d rpy_[kNumRollPitchYawTestCases]; // roll-pitch-yaw to be tested
};

TEST_F(RotationConversionTest, AxisQuat) {
  for (int i = 0; i < kNumAxisAngleTestCases; i++) {
    auto q = axis2quat(a_[i]);
    // Manually computes one quaternion corresponding to the axis-angle representation.
    Vector3d::PlainObject axis_normalized;
    NormalizeVector(a_[i].head<3>(), axis_normalized);
    Vector4d q_expected;
    q_expected(0) = std::cos(a_[i](3) / 2);
    q_expected.tail<3>() = std::sin(a_[i](3) / 2) * axis_normalized;

    EXPECT_TRUE(CompareMatrices(q_expected, q, 1E-10, MatrixCompareType::absolute)
        || CompareMatrices(q_expected, q, 1E-10, MatrixCompareType::absolute));
  }
}

TEST_F(RotationConversionTest, AxisRotmat) {
  // Manually computes the rotation matrix from axis-angle representation, using
  // Rodriguez's rotation formula
  // https://en.wikipedia.org/wiki/Rodrigues%27_rotation_formula
  for (int i = 0; i < kNumAxisAngleTestCases; i++) {
    auto axis_skew = VectorToSkewSymmetric(a_[i].head<3>());
    auto rotmat_expected = Matrix3d::Identity() + std::sin(a_[i](3)) * axis_skew
        + (1.0 - std::cos(a_[i](3))) * axis_skew * axis_skew;
    auto rotmat = axis2rotmat(a_[i]);
    EXPECT_TRUE(CompareMatrices(rotmat,
                                rotmat_expected,
                                1E-10,
                                MatrixCompareType::absolute));
  }
}

TEST_F(RotationConversionTest, QuatAxis) {
  for(int i = 0; i < kNumQuaternionTestCases; i++) {
    auto a = quat2axis(q_[i]);
    EXPECT_NEAR(a.head<3>().norm(), 1, 1E-6);
    auto quat_expected = axis2quat(a);
    EXPECT_TRUE(
        CompareMatrices(quat_expected, q_[i], 1E-10, MatrixCompareType::absolute)
            || CompareMatrices(quat_expected,
                               -q_[i],
                               1E-10,
                               MatrixCompareType::absolute));
  }
}

TEST_F(RotationConversionTest, QuatRotmat) {
  for(int i = 0; i < kNumQuaternionTestCases; i++) {
    // Computes the axis-angle representation, then converts to rotation matrix.
    auto rotmat_expected = axis2rotmat(quat2axis(q_[i]));
    auto rotmat = quat2rotmat(q_[i]);
    EXPECT_TRUE(CompareMatrices(rotmat_expected, rotmat, 1E-10, MatrixCompareType::absolute));
    // Compares quat2rotmat(q) and quat2rotmat(-q), the rotation matrices should
    // be the same.
    EXPECT_TRUE(CompareMatrices(rotmat, quat2rotmat(-q_[i]), 1E-10, MatrixCompareType::absolute));
  }
}

TEST_F(RotationConversionTest, QuatEigenQuaternion) {
  for(int i = 0; i < kNumQuaternionTestCases; i++) {
    Quaterniond eigenQuat = quat2eigenQuaternion(q_[i]);
    Matrix3d R_expected = quat2rotmat(q_[i]);
    Matrix3d R_eigen = eigenQuat.matrix();
    EXPECT_TRUE(
        CompareMatrices(R_expected,
                        R_eigen,
                        1e-6,
                        MatrixCompareType::absolute));
  }
}
TEST_F(RotationConversionTest, RotmatQuat) {
  // Knowing that quat2rotmat is correct, I will check if rotmat2quat is the
  // correct inversion function.
  for(int i = 0; i < kNumRotationMatrixTestCases; i++) {
    auto quat = rotmat2quat(R_[i]);
    auto rotmat_expected = quat2rotmat(quat);
    EXPECT_TRUE(CompareMatrices(R_[i],
                                rotmat_expected,
                                1E-10,
                                MatrixCompareType::absolute));
  }
}

TEST_F(RotationConversionTest, RotmatAxis) {
  // Knowing that axis2rotmat is correct, I will check if the rotmat2axis is the
  // correct inversion function.
  for(int i = 0; i < kNumRotationMatrixTestCases; i++) {
    auto a = rotmat2axis(R_[i]);
    EXPECT_NEAR(a.head<3>().norm(), 1, 1E-6);
    auto rotmat_expected = axis2rotmat(a);
    EXPECT_TRUE(CompareMatrices(R_[i],
                                rotmat_expected,
                                1E-10,
                                MatrixCompareType::absolute));
    // Computes the angle axis representation by first computing the quaternion,
    // and then converts the quaternion to angle axis.
    auto a_expected = quat2axis(rotmat2quat(R_[i]));
    EXPECT_TRUE(
        CompareMatrices(a, a_expected, 1E-10, MatrixCompareType::absolute)
            || CompareMatrices(a,
                               -a_expected,
                               1E-10,
                               MatrixCompareType::absolute));
  }
}

/*
TEST_F(RotationConversionTest, QuatRPY) {
  Vector3d rpy = quat2rpy(q_);
  Vector4d q_back = rpy2quat(rpy);
  EXPECT_NEAR(acos(std::abs(q_.transpose() * q_back)), 0.0, 1e-6);
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
