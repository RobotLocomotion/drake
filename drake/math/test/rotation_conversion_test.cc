/// @file
/// Tests that rotation conversion functions are inverses.

#include <cmath>

#include <Eigen/Dense>

#include "gtest/gtest.h"

#include "drake/math/axis_angle.h"
#include "drake/math/roll_pitch_yaw.h"
#include "drake/math/rotation_matrix.h"
#include "drake/math/quaternion.h"
#include "drake/util/eigen_matrix_compare.h"

using Eigen::Matrix3d;
using Eigen::Quaterniond;
using Eigen::Vector3d;
using Eigen::Vector4d;

namespace drake {

using util::CompareMatrices;
using util::MatrixCompareType;

namespace math {
namespace {

class RotationConversionTest : public ::testing::Test {
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

}  // namespace
}  // namespace math
}  // namespace drake
