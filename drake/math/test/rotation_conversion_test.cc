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
    initializeSingularEulerAnglesTestCase();
    initializeAngleAxisTestCases();
    initializeQuaternionTestCases();
    initializeRotationMatrixTestCases();
    initializeRollPitchYawTestCases();
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
    // 11. Degenerate case, pi/2 pitch angle

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
    // Case 11, pi/2 pitch angle
    auto singular_euler_a = Eigen::AngleAxis<double>(singular_euler_);
    a_[10] << singular_euler_a.axis(), singular_euler_a.angle();
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
    // 10. Degenerate case, pi/2 pitch for Euler angles

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
    // Case 10. pi/2 pitch for Euler angles.
    q_[9] << singular_euler_.w(), singular_euler_.x(), singular_euler_.y(), singular_euler_.z();
  }

  void initializeRotationMatrixTestCases() {
    // Tests conversion from rotation matrix, tests 7 cases:
    // 1. Degenerate case, no rotation
    // 2. Degenerate case, 180 degrees rotation around z axis
    // 3. Degenerate case, 180 degrees rotation around y axis
    // 4. Degenerate case, 180 degrees rotation around x axis
    // 5. Almost degenerate case, very small positive rotation angle
    // 6. Almost degenerate case, very small negative rotation angle
    // 7. Almost degenerate case, close to 180 degree rotation.
    // 8. Non-degenerage case
    // 9. Degenerate case, pitch=pi/2 for Euler angles.

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

    // Case 7. Almost degenerate case, close to 180 degree rotation
    R_[6] = axis2rotmat(Vector4d(0.5*sqrt(2), 0.4*sqrt(2), 0.3*sqrt(2), (1 - 1E-6)*M_PI));

    // Case 8. non-degenerate case
    R_[7] = axis2rotmat(Vector4d(0.5*sqrt(2), 0.4*sqrt(2), 0.3*sqrt(2), M_PI/6));

    // Case 9. pitch = pi/2 for Euler angles
    R_[8] = singular_euler_.toRotationMatrix();
  }

  void initializeRollPitchYawTestCases() {
    // Tests conversion from roll pitch yaw angles. Tests cases:
    // 1. no rotation
    // 2. Degenerate case, pitch = pi/2;
    // 3. Non-degenerate case.

    // Case 1, no rotation
    rpy_[0] <<0, 0, 0;
    // Case 2, pitch = pi/2;
    rpy_[1] << M_PI/3, M_PI/2, M_PI/4;
    // Case 3, non-degenerate case
    rpy_[2] << M_PI/3, M_PI/4, M_PI/6;
  };

  void initializeSingularEulerAnglesTestCase() {
    singular_euler_  = Eigen::AngleAxisd(M_PI/3, Vector3d::UnitZ())
        *Eigen::AngleAxisd(M_PI/2, Vector3d::UnitY())
        *Eigen::AngleAxisd(M_PI/4, Vector3d::UnitX());
  }

  Eigen::Quaterniond singular_euler_; // singular euler angles, pitch = pi/2
  static const int kNumAxisAngleTestCases = 11;
  static const int kNumQuaternionTestCases = 10;
  static const int kNumRotationMatrixTestCases = 9;
  static const int kNumRollPitchYawTestCases = 3;
  Vector4d a_[kNumAxisAngleTestCases]; // axis-angle representations to be tested
  Vector4d q_[kNumQuaternionTestCases];  // quaternions to be tested
  Matrix3d R_[kNumRotationMatrixTestCases];  // rotation matrices to be tested
  Vector3d rpy_[kNumRollPitchYawTestCases]; // roll-pitch-yaw to be tested
};

TEST_F(RotationConversionTest, AxisQuat) {
  for (int i = 0; i < kNumAxisAngleTestCases; i++) {
    // Computes the quaternion using Eigen geometry module, compares the result
    // with axis2quat
    auto a = Eigen::AngleAxis<double>(a_[i](3), a_[i].head<3>());
    auto q_eigen = Eigen::Quaternion<double>(a);
    Vector4d q_expected(q_eigen.w(), q_eigen.x(), q_eigen.y(), q_eigen.z());
    auto q = axis2quat(a_[i]);
    EXPECT_TRUE(CompareMatrices(q, q_expected, 1E-10, MatrixCompareType::absolute) || CompareMatrices(q, -q_expected, 1E-10, MatrixCompareType::absolute));
  }
}

TEST_F(RotationConversionTest, AxisRotmat) {
  for (int i = 0; i < kNumAxisAngleTestCases; i++) {
    // Manually computes the rotation matrix from axis-angle representation,
    // using Rodriguez's rotation formula
    // https://en.wikipedia.org/wiki/Rodrigues%27_rotation_formula
    auto axis_skew = VectorToSkewSymmetric(a_[i].head<3>());
    auto rotmat_expected = Matrix3d::Identity() + std::sin(a_[i](3)) * axis_skew
        + (1.0 - std::cos(a_[i](3))) * axis_skew * axis_skew;
    auto rotmat = axis2rotmat(a_[i]);
    EXPECT_TRUE(CompareMatrices(rotmat,
                                rotmat_expected,
                                1E-10,
                                MatrixCompareType::absolute));

    // Computes the rotation matrix using Eigen geometry module, compares the
    // result with axis2rotmat
    auto a = Eigen::AngleAxis<double>(a_[i](3), a_[i].head<3>());
    auto rotmat_expected2 = a.toRotationMatrix();
    EXPECT_TRUE(CompareMatrices(rotmat_expected2,
                                rotmat,
                                1E-10,
                                MatrixCompareType::absolute));
  }
}

TEST_F(RotationConversionTest, AxisRPY) {
  // Computes the rpy from Eigen geometry module, comapres the result with
  // axis2rpy
  for(int i = 0; i < kNumAxisAngleTestCases; i++) {
    auto a = Eigen::AngleAxisd(a_[i](3), a_[i].head<3>());
    auto euler_expected = a.toRotationMatrix().eulerAngles(2, 1, 0);
    Vector3d rpy_expected(euler_expected(2), euler_expected(1), euler_expected(0));
    auto rpy = axis2rpy(a_[i]);
    EXPECT_TRUE(CompareMatrices(rpy_expected, rpy, 1E-10, MatrixCompareType::absolute));
  }
}

TEST_F(RotationConversionTest, QuatAxis) {
  for(int i = 0; i < kNumQuaternionTestCases; i++) {
    // Computes the angle-axis representation using Eigen geometry module,
    // compares the result with quat2axis
    auto q = Eigen::Quaternion<double>(q_[i](0), q_[i](1), q_[i](2), q_[i](3));
    auto a_eigen_expected = Eigen::AngleAxis<double>(q);

    auto a = quat2axis(q_[i]);
    auto a_eigen = Eigen::AngleAxis<double>(a(3), a.head<3>());
    if(!a_eigen_expected.isApprox(a_eigen, 1E-3)) {
      std::cout<<"i:"<<i<<std::endl;
    }
    EXPECT_TRUE(a_eigen_expected.isApprox(a_eigen, 1E-3));
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

TEST_F(RotationConversionTest, RPYRotmat) {
  // Computes the rotation matrix by rotz(rpy(2))*roty(rpy(1))*rotx(rpy(0)),
  // then compares the result with rpy2rotmat
  for(int i = 0; i < kNumRollPitchYawTestCases; i++) {
    auto rotation_expected = Eigen::AngleAxisd(rpy_[i](2), Vector3d::UnitZ())
    * Eigen::AngleAxisd(rpy_[i](1), Vector3d::UnitY())
    * Eigen::AngleAxisd(rpy_[i](0), Vector3d::UnitX());
    auto rotmat = rpy2rotmat(rpy_[i]);
    EXPECT_TRUE(CompareMatrices(rotmat, rotation_expected.toRotationMatrix(), 1E-10, MatrixCompareType::absolute));
  }
}

TEST_F(RotationConversionTest, RPYAxis) {
  // Computes the angle-axis representation using Eigen's geometry module,
  // compares the result with rpy2axis
  for(int i = 0; i < kNumRollPitchYawTestCases; i++) {
    auto rotation_expected = Eigen::AngleAxisd(rpy_[i](2), Vector3d::UnitZ())
        * Eigen::AngleAxisd(rpy_[i](1), Vector3d::UnitY())
        * Eigen::AngleAxisd(rpy_[i](0), Vector3d::UnitX());
    auto angle_axis_expected = Eigen::AngleAxis<double>(rotation_expected);
    auto a = rpy2axis(rpy_[i]);
    Vector4d a_expected;
    a_expected << angle_axis_expected.axis(), angle_axis_expected.angle();
    EXPECT_TRUE(CompareMatrices(a, a_expected, 1E-10, MatrixCompareType::absolute) || CompareMatrices(a, -a_expected, 1E-10, MatrixCompareType::absolute));
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
