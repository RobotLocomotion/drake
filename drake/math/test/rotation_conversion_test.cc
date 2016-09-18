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
  return a1.toRotationMatrix().isApprox(a2.toRotationMatrix());
}

bool compareQuaternion(const Vector4d &q1, const Vector4d &q2) {
  return q1.isApprox(q2) | q1.isApprox(-q2);
}
Quaterniond eulerToQuaternion(const Vector3d euler) {
  // Compute the quaterion for euler angle using intrinsic z-y'-x''
  return AngleAxisd(euler(0), Vector3d::UnitZ())
      *AngleAxisd(euler(1), Vector3d::UnitY())
          *AngleAxisd(euler(2), Vector3d::UnitX());
}
bool compareEulerAngles(const Vector3d &euler_angles1, const Vector3d &euler_angles2) {
  auto q1 = eulerToQuaternion(euler_angles1);
  auto q2 = eulerToQuaternion(euler_angles2);
  return q1.isApprox(q2);
}

Vector4d eigenAxisToAxis(const AngleAxisd &a) {
  Vector4d ret;
  ret << a.axis(), a.angle();
  return ret;
}

Vector4d eigenQuaterniontoQuat(const Quaterniond& q) {
  return Vector4d(q.w(), q.x(), q.y(), q.z());
}

class RotationConversionTest : public ::testing::Test {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 protected:
  void SetUp() override {
    // We will test the following cases
    // Degenerate case, 0 rotation around x axis
    // Degenerate case, 0 rotation around y axis
    // Degenerate case, 0 rotation around z axis
    // Degenerate case, 0 rotation around a unit axis
    // Almost degenerate case, small positive rotation
    // Almost degenerate case, small negative rotation
    // Degenerate case, 180 rotation around x axis
    // Degenerate case, 180 rotation around y axis
    // Degenerate case, 180 rotation around z axis
    // Degenerare case, 180 rotation around a unit axis
    // Almost degenerate case, close to 180 rotation around a unit axis
    // Degenerate case, pitch = pi/2 for x-y'-z'' Euler angle
    // Almost degenerate case, pitch close to pi/2 for x-y'-z'' Euler angle
    // Non-degenerate case

    // 0 rotation around x axis
    test_orientation_.push_back(AngleAxisd(0, Vector3d::UnitX()));

    // 0 rotation around y axis
    test_orientation_.push_back(AngleAxisd(0, Vector3d::UnitY()));

    // 0 rotation around z axis
    test_orientation_.push_back(AngleAxisd(0, Vector3d::UnitZ()));

    // 0 rotation around a unit axis
    Vector3d axis(0.5*sqrt(2), 0.4*sqrt(2), 0.3*sqrt(2));
    test_orientation_.push_back(AngleAxisd(0, axis));

    // small positive rotation
    test_orientation_.push_back(AngleAxisd(1E-6, axis));

    // small negative rotation
    test_orientation_.push_back(AngleAxisd(-1E-6, axis));

    // 180 rotation around x axis
    test_orientation_.push_back(AngleAxisd(M_PI, Vector3d::UnitX()));

    // 180 rotation around y axis
    test_orientation_.push_back(AngleAxisd(M_PI, Vector3d::UnitY()));

    // 180 rotation around z axis
    test_orientation_.push_back(AngleAxisd(M_PI, Vector3d::UnitZ()));

    // close to 180 rotation around a unit axis
    test_orientation_.push_back(AngleAxisd((1-1E-6)*M_PI, axis));

    // pitch = pi/2
    AngleAxisd a1(AngleAxisd(M_PI/3, Vector3d::UnitZ())
                      *AngleAxisd(M_PI/2, Vector3d::UnitY())
                      *AngleAxisd(M_PI/4, Vector3d::UnitX()));
    test_orientation_.push_back(a1);

    // pitch close to pi/2
    AngleAxisd a2(AngleAxisd(M_PI/3, Vector3d::UnitZ())
                      *AngleAxisd(M_PI/2-1E-6*M_PI, Vector3d::UnitY())
                      *AngleAxisd(M_PI/4, Vector3d::UnitX()));
    test_orientation_.push_back(a2);

    // pitch close to pi/2
    AngleAxisd a3(AngleAxisd(M_PI/3, Vector3d::UnitZ())
                      *AngleAxisd(M_PI/2+1E-6*M_PI, Vector3d::UnitY())
                      *AngleAxisd(M_PI/4, Vector3d::UnitX()));
    test_orientation_.push_back(a3);

    // non-degenerate case
    test_orientation_.push_back(AngleAxisd(M_PI/3, axis));
  }
  std::vector<AngleAxisd> test_orientation_; // test cases
};

TEST_F(RotationConversionTest, AxisQuat) {
  for (const auto &ai_eigen:test_orientation_) {
    // Compute the quaternion using Eigen geometry module, compare the result
    // with axis2quat
    auto ai = eigenAxisToAxis(ai_eigen);
    auto q_eigen_expected = Eigen::Quaternion<double>(ai_eigen);
    auto q = axis2quat(ai);
    auto q_eigen = quat2eigenQuaternion(q);
    EXPECT_TRUE(q_eigen.isApprox(q_eigen_expected));
    // axis2quat should be the inversion of quat2axis
    auto a_expected = quat2axis(q);
    AngleAxisd a_eigen_expected(a_expected(3), a_expected.head<3>());
    EXPECT_TRUE(compareAngleAxis(ai_eigen, a_eigen_expected));
  }
}

TEST_F(RotationConversionTest, AxisRotmat) {
  for (const auto &ai_eigen:test_orientation_) {
    // Manually computes the rotation matrix from axis-angle representation,
    // using Rodriguez's rotation formula
    // https://en.wikipedia.org/wiki/Rodrigues%27_rotation_formula
    auto ai = eigenAxisToAxis(ai_eigen);
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
    auto rotmat_expected2 = ai_eigen.toRotationMatrix();
    EXPECT_TRUE(CompareMatrices(rotmat_expected2,
                                rotmat,
                                1E-10,
                                MatrixCompareType::absolute));

    // axis2rotmat should be the inversion of rotmat2axis
    auto a_expected = rotmat2axis(rotmat);
    AngleAxisd a_eigen_expected(a_expected(3), a_expected.head<3>());
    EXPECT_TRUE(compareAngleAxis(ai_eigen, a_eigen_expected));
  }
}

TEST_F(RotationConversionTest, AxisRPY) {
  // Compute the rpy from Eigen geometry module, comapres the result with
  // axis2rpy
  for(const auto ai_eigen:test_orientation_) {
    auto euler_expected = ai_eigen.toRotationMatrix().eulerAngles(2, 1, 0);
    auto ai = eigenAxisToAxis(ai_eigen);
    auto rpy = axis2rpy(ai);
    Vector3d euler(rpy(2), rpy(1), rpy(0));
    EXPECT_TRUE(compareEulerAngles(euler, euler_expected));
    // axis2rpy should be the inversion of rpy2axis
    auto a_expected = rpy2axis(rpy);
    AngleAxisd a_eigen_expected(a_expected(3), a_expected.head<3>());
    EXPECT_TRUE(compareAngleAxis(ai_eigen, a_eigen_expected));
  }
}

TEST_F(RotationConversionTest, QuatAxis) {
  for(const auto &ai:test_orientation_) {
    // Compute the angle-axis representation using Eigen geometry module,
    // compare the result with quat2axis
    Quaterniond qi_eigen(ai);
    Vector4d qi = eigenQuaterniontoQuat(qi_eigen);
    auto a_eigen_expected = Eigen::AngleAxis<double>(qi_eigen);

    auto a = quat2axis(qi);
    auto a_eigen = Eigen::AngleAxis<double>(a(3), a.head<3>());
    EXPECT_TRUE(compareAngleAxis(a_eigen, a_eigen_expected));
    // quat2axis should be the inversion of axis2quat
    auto quat_expected = axis2quat(a);
    EXPECT_TRUE(compareQuaternion(qi, quat_expected));
  }
}

TEST_F(RotationConversionTest, QuatRotmat) {
  for(const auto &ai:test_orientation_) {
    // Compute the rotation matrix using Eigen geometry module, compare the
    // result with quat2rotmat
    Quaterniond qi_eigen(ai);
    Vector4d qi = eigenQuaterniontoQuat(qi_eigen);
    auto rotmat_expected = qi_eigen.toRotationMatrix();
    auto rotmat = quat2rotmat(qi);
    EXPECT_TRUE(CompareMatrices(rotmat_expected, rotmat, 1E-10, MatrixCompareType::absolute));
    // quat2rotmat should be the inversion of rotmat2quat
    auto quat_expected = rotmat2quat(rotmat);
    EXPECT_TRUE(compareQuaternion(qi, quat_expected));
  }
}

TEST_F(RotationConversionTest, QuatRPY) {
  // Compute the roll pitch yaw angle using Eigen geometry module, compare the
  // result with quat2rpy
  for (const auto &ai:test_orientation_) {
    Quaterniond qi_eigen(ai);
    Vector4d qi = eigenQuaterniontoQuat(qi_eigen);
    auto ea_eigen = qi_eigen.toRotationMatrix().eulerAngles(2, 1, 0);
    auto rpy = quat2rpy(qi);
    Vector3d ea(rpy(2), rpy(1), rpy(0));
    EXPECT_TRUE(compareEulerAngles(ea, ea_eigen));
    // quat2rpy should be the inversion of rpy2quat
    auto quat_expected = rpy2quat(rpy);
    EXPECT_TRUE(compareQuaternion(qi, quat_expected));
  }
}

TEST_F(RotationConversionTest, QuatEigenQuaternion) {
  for(const auto &ai:test_orientation_) {
    Quaterniond qi_eigen(ai);
    Vector4d qi = eigenQuaterniontoQuat(qi_eigen);
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
  for(const auto &ai:test_orientation_) {
    auto Ri = ai.toRotationMatrix();
    auto quat = rotmat2quat(Ri);
    auto quat_eigen = Quaterniond(quat(0), quat(1), quat(2), quat(3));
    auto quat_expected_eigen = Quaterniond(Ri);
    EXPECT_TRUE(quat_eigen.isApprox(quat_expected_eigen));
    // rotmat2quat should be the inversion of quat2rotmat
    auto rotmat_expected = quat2rotmat(quat);
    EXPECT_TRUE(Ri.isApprox(rotmat_expected));
  }
}

TEST_F(RotationConversionTest, RotmatAxis) {
  // Compute angle-axis using Eigen geometry module, compare the result with
  // rotmat2axis
  for(const auto &ai:test_orientation_) {
    auto Ri = ai.toRotationMatrix();
    auto a_eigen_expected = AngleAxisd(Ri);
    auto a = rotmat2axis(Ri);
    auto a_eigen = AngleAxisd(a(3), a.head<3>());
    EXPECT_TRUE(compareAngleAxis(a_eigen, a_eigen_expected));
    // rotmat2axis should be the inversion of axis2rotmat
    auto rotmat_expected = axis2rotmat(a);
    EXPECT_TRUE(Ri.isApprox(rotmat_expected));
  }
}

TEST_F(RotationConversionTest, RotmatRPY) {
  // Compute roll pitch yaw using Eigen geometry module, compare the result with
  // rotmat2rpy
  for(const auto &ai:test_orientation_) {
    auto Ri = ai.toRotationMatrix();
    auto ea_expected = Ri.eulerAngles(2, 1, 0);
    Vector3d rpy_expected(ea_expected(2), ea_expected(1), ea_expected(0));
    auto rpy = rotmat2rpy(Ri);
    Vector3d ea(rpy(2), rpy(1), rpy(0));
    EXPECT_TRUE(compareEulerAngles(ea, ea_expected));
    // rotmat2rpy should be the inversion of rpy2rotmat
    auto rotmat_expected = rpy2rotmat(rpy);
    EXPECT_TRUE(Ri.isApprox(rotmat_expected));
  }
}
TEST_F(RotationConversionTest, RPYRotmat) {
  // Compute the rotation matrix by rotz(rpy(2))*roty(rpy(1))*rotx(rpy(0)),
  // then compare the result with rpy2rotmat
  for(const auto &ai:test_orientation_) {
    auto euler = ai.toRotationMatrix().eulerAngles(2, 1, 0);
    Vector3d rpyi(euler(2), euler(1), euler(0));
    auto rotation_expected = Eigen::AngleAxisd(rpyi(2), Vector3d::UnitZ())
    * Eigen::AngleAxisd(rpyi(1), Vector3d::UnitY())
    * Eigen::AngleAxisd(rpyi(0), Vector3d::UnitX());
    auto rotmat = rpy2rotmat(rpyi);
    EXPECT_TRUE(CompareMatrices(rotmat, rotation_expected.toRotationMatrix(), 1E-10, MatrixCompareType::absolute));
    // rpy2rotmat should be the inversion of rotmat2rpy
    Vector3d rpy_expected = rotmat2rpy(rotmat);
    Vector3d euler_expected(rpy_expected(2), rpy_expected(1), rpy_expected(0));
    EXPECT_TRUE(compareEulerAngles(euler, euler_expected));
  }
}

TEST_F(RotationConversionTest, RPYAxis) {
  // Compute the angle-axis representation using Eigen's geometry module,
  // compare the result with rpy2axis
  for(const auto &ai:test_orientation_) {
    auto euler = ai.toRotationMatrix().eulerAngles(2, 1, 0);
    Vector3d rpyi(euler(2), euler(1), euler(0));
    auto rotation_expected = Eigen::AngleAxisd(rpyi(2), Vector3d::UnitZ())
        * Eigen::AngleAxisd(rpyi(1), Vector3d::UnitY())
        * Eigen::AngleAxisd(rpyi(0), Vector3d::UnitX());
    auto a_eigen_expected = Eigen::AngleAxis<double>(rotation_expected);
    auto a = rpy2axis(rpyi);
    AngleAxisd a_eigen(a(3), a.head<3>());
    EXPECT_TRUE(compareAngleAxis(a_eigen, a_eigen_expected));

    // Compute rpy2axis should be the inversion of axis2rpy
    Vector3d rpy_expected = axis2rpy(a);
    Vector3d euler_expected (rpy_expected(2), rpy_expected(1), rpy_expected(0));
    EXPECT_TRUE(compareEulerAngles(euler, euler_expected));
  }
}

TEST_F(RotationConversionTest, RPYQuat) {
  // Compute the quaternion representation using Eigen's geometry model,
  // compare the result with rpy2quat
  for(const auto &ai:test_orientation_) {
    auto euler = ai.toRotationMatrix().eulerAngles(2, 1, 0);
    Vector3d rpyi(euler(2), euler(1), euler(0));
    auto quat_eigen_expected = eulerToQuaternion(euler);
    auto quat = rpy2quat(rpyi);
    auto quat_eigen = quat2eigenQuaternion(quat);
    EXPECT_TRUE(quat_eigen.isApprox((quat_eigen_expected)));
    // rpy2quat should be the inversion of quat2rpy
    auto rpy_expected = quat2rpy(quat);
    Vector3d euler_expected(rpy_expected(2), rpy_expected(1), rpy_expected(0));
    EXPECT_TRUE(compareEulerAngles(euler, euler_expected));
  }
}
/*
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
