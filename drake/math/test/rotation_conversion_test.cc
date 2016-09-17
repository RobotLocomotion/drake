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
using Eigen::AngleAxis;
using Eigen::AngleAxisd;
using Eigen::Quaternion;
using Eigen::Quaterniond;
using std::sin;
using std::cos;

namespace drake {
namespace math {
namespace {
// Compare the axis and angle separetly. We are not using AngleAxis.isApprox()
// because a difference of 2*PI in rotation angle matters; Eigen thinks a
// rotation of 0 around axis @p a is different from a rotation of 2*pi around
// the same axis @p a.
bool compareAngleAxis(const AngleAxisd &a1, const AngleAxisd &a2) {
  bool ret = CompareMatrices(a1.axis(), a2.axis(), 1E-10, MatrixCompareType::absolute);
  ret &= (std::abs(sin(a1.angle()) - sin(a2.angle())) < 1E-10) & (std::abs(cos(a1.angle()) - cos(a2.angle())) < 1E-10);
  return ret;
}
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
    // 12. Almost degenerate case, pitch close to pi/2 for Euler angles.

    // Case 1. no rotation around x axis.
    a_.push_back(Vector4d(1, 0, 0, 0));
    // Case 2. no rotation around y axis.
    a_.push_back(Vector4d(0, 1, 0, 0));
    // Case 3. no rotation around z axis.
    a_.push_back(Vector4d(0, 0, 1, 0));
    // Case 4. 180 degrees rotation around x axis.
    a_.push_back(Vector4d(1, 0, 0, M_PI));
    // Case 5, 180 degrees rotation around y axis.
    a_.push_back(Vector4d(0, 1, 0, M_PI));
    // Case 6, 180 degrees rotation around z axis.
    a_.push_back(Vector4d(0, 0, 1, M_PI));
    // Case 7, small positive rotation angle
    a_.push_back(Vector4d(0.5*sqrt(2), 0.4*sqrt(2), 0.3*sqrt(2), 1E-6*M_PI));
    // Case 8, small negative rotation angle
    a_.push_back(Vector4d(0.5*sqrt(2), 0.4*sqrt(2), 0.3*sqrt(2), -1E-6*M_PI));
    // Case 9, close to 180 rotation angle
    a_.push_back(Vector4d(0.5*sqrt(2), 0.4*sqrt(2), 0.3*sqrt(2), (1-1E-6)*M_PI));
    // Case 10, non-degenerate case
    a_.push_back(Vector4d(0.5*sqrt(2), 0.4*sqrt(2), 0.3*sqrt(2), M_PI/4));
    // Case 11, pi/2 pitch angle
    for(const auto eai:singular_euler_) {
      auto singular_euler_a_eigen = Eigen::AngleAxis<double>(eai);
      Vector4d singular_euler_a;
      singular_euler_a
          << singular_euler_a_eigen.axis(), singular_euler_a_eigen.angle();
      a_.push_back(singular_euler_a);
    }
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
    // 11. Almost degenerate case, pitch close to pi/2 for Euler angles

    // Case 1. no rotation.
    q_.push_back(Vector4d(1, 0, 0, 0));
    // Case 2. no rotation.
    q_.push_back(Vector4d(-1, 0, 0, 0));
    // Case 3. 180 rotation around x axis.
    q_.push_back(Vector4d(0, 1, 0, 0));
    // Case 4. 180 rotation around y axis.
    q_.push_back(Vector4d(0, 0, 1, 0));
    // Case 5. 180 rotation around z axis.
    q_.push_back(Vector4d(0, 0, 0, 1));
    // Case 6. Almost degenerate case, small positive rotation angle.
    q_.push_back(Vector4d(cos(1E-6*M_PI), 0.5*sqrt(2)*sin(1E-6*M_PI), 0.4*sqrt(2)*sin(1E-6*M_PI), 0.3*sqrt(2)*sin(1E-6*M_PI)));
    // Case 7. Almost degenerate case, small negative rotation angle.
    q_.push_back(Vector4d(cos(-1E-6*M_PI), 0.5*sqrt(2)*sin(-1E-6*M_PI), 0.4*sqrt(2)*sin(-1E-6*M_PI), 0.3*sqrt(2)*sin(-1E-6*M_PI)));
    // Case 8. Almost degenerate case, close to 180 degree rotation.
    q_.push_back(Vector4d(cos((1-1E-6)*M_PI), 0.5*sqrt(2)*sin((1-1E-6)*M_PI), 0.4*sqrt(2)*sin((1-1E-6)*M_PI), 0.3*sqrt(2)*sin((1-1E-6)*M_PI)));
    // Case 9. Non-degenerate case.
    q_.push_back(Vector4d(1/sqrt(2), 0.5, 1/sqrt(6), 1/sqrt(12)));
    // Case 10. pi/2 pitch for Euler angles.
    for(const auto &eai:singular_euler_) {
      q_.push_back(Vector4d(eai.w(),
                            eai.x(),
                            eai.y(),
                            eai.z()));
    }
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
    // 10. Almost degenerate case, pitch close to pi/2 for Euler angles z-y'-x''

    // Case 1. no rotation
    R_.push_back(Matrix3d::Identity());

    // Case 2. 180 degrees rotation around z axis
    Matrix3d Ri;
    Ri = Matrix3d::Zero();
    Ri(0, 0) = -1;
    Ri(1, 1) = -1;
    Ri(2, 2) = 1;
    R_.push_back(Ri);

    // Case 3. 180 degrees rotation around y axis
    Ri.setZero();
    Ri(0, 0) = -1;
    Ri(1, 1) = 1;
    Ri(2, 2) = -1;
    R_.push_back(Ri);

    // Case 4. 180 degrees rotation around x axis
    Ri.setZero();
    Ri(0, 0) = 1;
    Ri(1, 1) = -1;
    Ri(2, 2) = -1;
    R_.push_back(Ri);

    // Case 5. small positive rotation angle
    R_.push_back(axis2rotmat(Vector4d(0.5*sqrt(2), 0.4*sqrt(2), 0.3*sqrt(2), 1E-6*M_PI)));

    // Case 6. small negative rotation angle
    R_.push_back(axis2rotmat(Vector4d(0.5*sqrt(2), 0.4*sqrt(2), 0.3*sqrt(2), -1E-6*M_PI)));

    // Case 7. Almost degenerate case, close to 180 degree rotation
    R_.push_back(axis2rotmat(Vector4d(0.5*sqrt(2), 0.4*sqrt(2), 0.3*sqrt(2), (1 - 1E-6)*M_PI)));

    // Case 8. non-degenerate case
    R_.push_back(axis2rotmat(Vector4d(0.5*sqrt(2), 0.4*sqrt(2), 0.3*sqrt(2), M_PI/6)));

    // Case 9 pitch = pi/2 for Euler angles
    for(const auto &eai:singular_euler_) {
      R_.push_back(eai.toRotationMatrix());
    }
  }

  void initializeRollPitchYawTestCases() {
    // Tests conversion from roll pitch yaw angles. Tests cases:
    // 1. no rotation
    // 2. Degenerate case, pitch = pi/2;
    // 3. Non-degenerate case.

    // Case 1, no rotation
    rpy_.push_back(Vector3d::Zero());
    // Case 2, pitch = pi/2;
    rpy_.push_back(Vector3d(M_PI/3, M_PI/2, M_PI/4));
    // Case 3, non-degenerate case
    rpy_.push_back(Vector3d(M_PI/3, M_PI/4, M_PI/6));
  };

  void initializeSingularEulerAnglesTestCase() {
    singular_euler_.push_back(Eigen::AngleAxisd(M_PI/3, Vector3d::UnitZ())
        *Eigen::AngleAxisd(M_PI/2, Vector3d::UnitY())
        *Eigen::AngleAxisd(M_PI/4, Vector3d::UnitX()));
    singular_euler_.push_back(Eigen::AngleAxisd(M_PI/3, Vector3d::UnitZ())
                         *Eigen::AngleAxisd(M_PI/2+1E-6*M_PI, Vector3d::UnitY())
                         *Eigen::AngleAxisd(M_PI/4, Vector3d::UnitX()));
  }

  std::vector<Quaterniond> singular_euler_; // singular euler angles, pitch = pi/2
  std::vector<Vector4d> a_; // axis-angle representations to be tested
  std::vector<Vector4d> q_; // quaternions to be tested
  std::vector<Matrix3d> R_; // rotation matrices to be tested
  std::vector<Vector3d> rpy_; // roll-pitch-yaw to be tested
};

TEST_F(RotationConversionTest, AxisQuat) {
  for (const auto &ai:a_) {
    // Compute the quaternion using Eigen geometry module, compare the result
    // with axis2quat
    auto a = Eigen::AngleAxis<double>(ai(3), ai.head<3>());
    auto q_eigen = Eigen::Quaternion<double>(a);
    Vector4d q_expected(q_eigen.w(), q_eigen.x(), q_eigen.y(), q_eigen.z());
    auto q = axis2quat(ai);
    EXPECT_TRUE(CompareMatrices(q, q_expected, 1E-10, MatrixCompareType::absolute) || CompareMatrices(q, -q_expected, 1E-10, MatrixCompareType::absolute));
  }
}

TEST_F(RotationConversionTest, AxisRotmat) {
  for (const auto &ai:a_) {
    // Manually computes the rotation matrix from axis-angle representation,
    // using Rodriguez's rotation formula
    // https://en.wikipedia.org/wiki/Rodrigues%27_rotation_formula
    auto axis_skew = VectorToSkewSymmetric(ai.head<3>());
    auto rotmat_expected = Matrix3d::Identity() + std::sin(ai(3)) * axis_skew
        + (1.0 - std::cos(ai(3))) * axis_skew * axis_skew;
    auto rotmat = axis2rotmat(ai);
    EXPECT_TRUE(CompareMatrices(rotmat,
                                rotmat_expected,
                                1E-10,
                                MatrixCompareType::absolute));

    // Compute the rotation matrix using Eigen geometry module, compare the
    // result with axis2rotmat
    auto a = Eigen::AngleAxis<double>(ai(3), ai.head<3>());
    auto rotmat_expected2 = a.toRotationMatrix();
    EXPECT_TRUE(CompareMatrices(rotmat_expected2,
                                rotmat,
                                1E-10,
                                MatrixCompareType::absolute));
  }
}

TEST_F(RotationConversionTest, AxisRPY) {
  // Compute the rpy from Eigen geometry module, comapres the result with
  // axis2rpy
  for(const auto ai:a_) {
    auto a = Eigen::AngleAxisd(ai(3), ai.head<3>());
    auto euler_expected = a.toRotationMatrix().eulerAngles(2, 1, 0);
    Vector3d rpy_expected(euler_expected(2), euler_expected(1), euler_expected(0));
    auto rpy = axis2rpy(ai);
    auto qi = axis2quat(ai);
    auto q_eigen = Quaterniond(a);
    if(!q_eigen.isApprox(Quaterniond(qi(0), qi(1), qi(2), qi(3)))) {
      std::cout<<"quaternion not match"<<std::endl;
    }

    //std::cout<<qi<<" "<<q_eigen<<std::endl;
    EXPECT_TRUE(CompareMatrices(rpy_expected, rpy, 1E-10, MatrixCompareType::absolute));
  }
}

TEST_F(RotationConversionTest, QuatAxis) {
  for(const auto &qi:q_) {
    // Compute the angle-axis representation using Eigen geometry module,
    // compare the result with quat2axis
    auto q = Eigen::Quaternion<double>(qi(0), qi(1), qi(2), qi(3));
    auto a_eigen_expected = Eigen::AngleAxis<double>(q);

    auto a = quat2axis(qi);
    auto a_eigen = Eigen::AngleAxis<double>(a(3), a.head<3>());
    EXPECT_TRUE(compareAngleAxis(a_eigen, a_eigen_expected));
  }
}

TEST_F(RotationConversionTest, QuatRotmat) {
  for(const auto &qi:q_) {
    // Compute the rotation matrix using Eigen geometry module, compare the
    // result with quat2rotmat
    auto q_eigen = Quaterniond(qi(0), qi(1), qi(2), qi(3));
    auto rotmat_expected = q_eigen.toRotationMatrix();
    auto rotmat = quat2rotmat(qi);
    EXPECT_TRUE(CompareMatrices(rotmat_expected, rotmat, 1E-10, MatrixCompareType::absolute));
    // compare quat2rotmat(q) and quat2rotmat(-q), the rotation matrices should
    // be the same.
    EXPECT_TRUE(CompareMatrices(rotmat, quat2rotmat(-qi), 1E-10, MatrixCompareType::absolute));
  }
}

TEST_F(RotationConversionTest, QuatRPY) {
  // Compute the roll pitch yaw angle using Eigen geometry module, compare the
  // result with quat2rpy
  for (const auto &qi:q_) {
    auto q_eigen = Quaterniond(qi(0), qi(1), qi(2), qi(3));
    auto ea_eigen = q_eigen.toRotationMatrix().eulerAngles(2, 1, 0);
    Vector3d rpy_expected(ea_eigen(2), ea_eigen(1), ea_eigen(0));
    auto rpy = quat2rpy(qi);
    EXPECT_TRUE(CompareMatrices(rpy, rpy_expected, 1E-10, MatrixCompareType::absolute));
  }
}

TEST_F(RotationConversionTest, QuatEigenQuaternion) {
  for(const auto &qi:q_) {
    Quaterniond eigenQuat = quat2eigenQuaternion(qi);
    Matrix3d R_expected = quat2rotmat(qi);
    Matrix3d R_eigen = eigenQuat.matrix();
    EXPECT_TRUE(
        CompareMatrices(R_expected,
                        R_eigen,
                        1e-6,
                        MatrixCompareType::absolute));
  }
}
TEST_F(RotationConversionTest, RotmatQuat) {
  // Compute the quaternion using Eigen geomery module, compare the result with
  // rotmat2quat
  for(const auto &Ri:R_) {
    auto quat = rotmat2quat(Ri);
    auto quat_eigen = Quaterniond(quat(0), quat(1), quat(2), quat(3));
    auto quat_expected_eigen = Quaterniond(Ri);
    EXPECT_TRUE(quat_eigen.isApprox(quat_expected_eigen));
  }
}

TEST_F(RotationConversionTest, RotmatAxis) {
  // Compute angle-axis using Eigen geometry module, compare the result with
  // rotmat2axis
  for(const auto &Ri:R_) {
    auto a_eigen_expected = AngleAxisd(Ri);
    auto a = rotmat2axis(Ri);
    auto a_eigen = AngleAxisd(a(3), a.head<3>());
    EXPECT_TRUE(compareAngleAxis(a_eigen, a_eigen_expected));
  }
}

TEST_F(RotationConversionTest, RotmatRPY) {
  // Compute roll pitch yaw using Eigen geometry module, compare the result with
  // rotmat2rpy
  for(const auto &Ri:R_) {
    auto ea_expected = Ri.eulerAngles(2, 1, 0);
    Vector3d rpy_expected(ea_expected(2), ea_expected(1), ea_expected(0));
    auto rpy = rotmat2rpy(Ri);
    EXPECT_TRUE(CompareMatrices(rpy, rpy_expected, 1E-10, MatrixCompareType::absolute));
  }
}
TEST_F(RotationConversionTest, RPYRotmat) {
  // Compute the rotation matrix by rotz(rpy(2))*roty(rpy(1))*rotx(rpy(0)),
  // then compare the result with rpy2rotmat
  for(const auto &rpyi:rpy_) {
    auto rotation_expected = Eigen::AngleAxisd(rpyi(2), Vector3d::UnitZ())
    * Eigen::AngleAxisd(rpyi(1), Vector3d::UnitY())
    * Eigen::AngleAxisd(rpyi(0), Vector3d::UnitX());
    auto rotmat = rpy2rotmat(rpyi);
    EXPECT_TRUE(CompareMatrices(rotmat, rotation_expected.toRotationMatrix(), 1E-10, MatrixCompareType::absolute));
  }
}

TEST_F(RotationConversionTest, RPYAxis) {
  // Compute the angle-axis representation using Eigen's geometry module,
  // compare the result with rpy2axis
  for(const auto &rpyi:rpy_) {
    auto rotation_expected = Eigen::AngleAxisd(rpyi(2), Vector3d::UnitZ())
        * Eigen::AngleAxisd(rpyi(1), Vector3d::UnitY())
        * Eigen::AngleAxisd(rpyi(0), Vector3d::UnitX());
    auto a_eigen_expected = Eigen::AngleAxis<double>(rotation_expected);
    auto a = rpy2axis(rpyi);
    AngleAxisd a_eigen(a(3), a.head<3>());
    EXPECT_TRUE(compareAngleAxis(a_eigen, a_eigen_expected));
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
