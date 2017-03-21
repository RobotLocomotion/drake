/// @file
/// Tests that rotation conversion functions are inverses.

#include <cmath>
#include <iostream>

#include <Eigen/Dense>
#include <gtest/gtest.h>

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
using std::numeric_limits;

namespace drake {
namespace math {
namespace {

// kSweepSize is an even number, so that no samples are taken at zero. This
// test scales as O(N^4) in sweep size, so be cautious about turning it up!
const int kSweepSize = 6;

Vector4d EigenQuaternionToOrderWXYZ(const Quaterniond& q) {
  return Vector4d(q.w(), q.x(), q.y(), q.z());
}

bool check_rpy_range(const Vector3d& rpy) {
  return rpy(0) <= M_PI && rpy(0) >= -M_PI && rpy(1) <= M_PI / 2 &&
         rpy(1) >= -M_PI / 2 && rpy(2) <= M_PI && rpy(2) >= -M_PI;
}
// Compare the equivalent rotation matrix.
// Note that we are not comparing the axis-angle directly. This is because the
// axis-angle has singularities around 0 degree rotation and 180 degree rotation
// So two axis-angles that are slightly different when the angle is close to 0,
// their equivalent rotation matrices are almost the same.
bool AreAngleAxisForSameOrientation(const AngleAxisd& a1,
                                    const AngleAxisd& a2) {
  return a1.toRotationMatrix().isApprox(a2.toRotationMatrix());
}

bool AreQuaternionsForSameOrientation(const Vector4d& q1, const Vector4d& q2,
                                      double precision = 1E-12) {
  // The same orientation is described by both a quaternion and the negative of
  // that quaternion.
  return q1.isApprox(q2, precision) | q1.isApprox(-q2, precision);
}
Quaterniond BodyZYXAnglesToEigenQuaternion(const Vector3d bodyZYX_angles) {
  // Compute the quaterion for euler angle using intrinsic z-y'-x''.
  // Uses Eigen functionality (including overloaded operators *, etc).
  return AngleAxisd(bodyZYX_angles(0), Vector3d::UnitZ()) *
         AngleAxisd(bodyZYX_angles(1), Vector3d::UnitY()) *
         AngleAxisd(bodyZYX_angles(2), Vector3d::UnitX());
}

bool AreEulerAnglesForSameOrientation(const Vector3d& euler_angles1,
                                      const Vector3d& euler_angles2,
                                      double precision = 1E-12) {
  auto q1 = BodyZYXAnglesToEigenQuaternion(euler_angles1);
  auto q2 = BodyZYXAnglesToEigenQuaternion(euler_angles2);
  return AreQuaternionsForSameOrientation(EigenQuaternionToOrderWXYZ(q1),
                                          EigenQuaternionToOrderWXYZ(q2),
                                          precision);
}

bool AreRollPitchYawForSameOrientation(const Vector3d& rpy1,
                                       const Vector3d& rpy2) {
  Vector3d euler_angles1(rpy1(2), rpy1(1), rpy1(0));
  Vector3d euler_angles2(rpy2(2), rpy2(1), rpy2(0));
  // Note: When pitch is close to PI/2 or -PI/2, derivative calculations for
  // Euler angle can encounter numerical problems.  However, although values
  // of angles may "jump around" (hence, difficult derivatives), the angles'
  // values should be accurately reproduced.
  const double precision = 1E-13;
  return AreEulerAnglesForSameOrientation(euler_angles1, euler_angles2,
                                          precision);
}

Vector4d EigenAngleAxisToDrakeAxisAngle(const AngleAxisd& eigenAngleAxis) {
  Vector4d drakeAxisAngle;
  drakeAxisAngle << eigenAngleAxis.axis(), eigenAngleAxis.angle();
  return drakeAxisAngle;
}

Matrix3d CalcRotationMatrixAboutZ(double a) {
  // Returns 3 x 3 R_AB matrix where vA = R_AB * v_B.
  // vB is a vector expressed in basis B and vA is the equivalent vector, but
  // expressed in basis A.
  // basis B is obtained by rotating basis A about Z axis by angle a.
  Matrix3d ret;
  ret << cos(a), -sin(a), 0, sin(a), cos(a), 0, 0, 0, 1;
  return ret;
}

Matrix3d CalcRotationMatrixAboutY(double b) {
  // Returns 3 x 3 R_AB matrix where vA = R_AB * v_B.
  // vB is a vector expressed in basis B and vA is the equivalent vector, but
  // expressed in basis A.
  // basis B is obtained by rotating basis A about Y axis by angle b.
  Matrix3d ret;
  ret << cos(b), 0, sin(b), 0, 1, 0, -sin(b), 0, cos(b);
  return ret;
}

Matrix3d CalcRotationMatrixAboutX(double c) {
  // Returns 3 x 3 R_AB matrix where vA = R_AB * v_B.
  // vB is a vector expressed in basis B and vA is the equivalent vector, but
  // expressed in basis A.
  // basis B is obtained by rotating basis A about X axis by angle c.
  Matrix3d ret;
  ret << 1, 0, 0, 0, cos(c), -sin(c), 0, sin(c), cos(c);
  return ret;
}
GTEST_TEST(EigenEulerAngleTest, BodyXYZ) {
  // Verify ea = Eigen::eulerAngles(0, 1, 2) returns Euler angles about
  // Body-fixed x-y'-z'' axes by [ea(0), ea(1), ea(2)].
  Vector3d input_angles(0.5, 0.4, 0.3);
  Matrix3d bodyXYZ_rotmat = CalcRotationMatrixAboutX(input_angles(0)) *
                            CalcRotationMatrixAboutY(input_angles(1)) *
                            CalcRotationMatrixAboutZ(input_angles(2));
  auto output_angles = bodyXYZ_rotmat.eulerAngles(0, 1, 2);
  // input_angles.isApprox(output_angles) is a valid test (rathan than
  // comparing the converted quaternions) since all the angles are between
  // 0 and PI/2.
  EXPECT_TRUE(input_angles.isApprox(output_angles));
}

GTEST_TEST(EigenEulerAngleTest, BodyZYX) {
  // Verify ea = Eigen::eulerAngles(2, 1, 0) returns Euler angles about
  // Body-fixed z-y'-x'' axes by [ea(0), ea(1), ea(2)].
  Vector3d input_angles(0.5, 0.4, 0.3);
  Matrix3d bodyZYX_rotmat = CalcRotationMatrixAboutZ(input_angles(0)) *
                            CalcRotationMatrixAboutY(input_angles(1)) *
                            CalcRotationMatrixAboutX(input_angles(2));
  auto output_angles = bodyZYX_rotmat.eulerAngles(2, 1, 0);
  // input_angles.isApprox(output_angles) is a valid test (rathan than
  // comparing the converted quaternions) since all the angles are between
  // 0 and PI/2.
  EXPECT_TRUE(input_angles.isApprox(output_angles));
}

GTEST_TEST(EigenEulerAngleTest, BodyZYZ) {
  // Verify ea = Eigen::eulerAngles(2, 1, 0) returns Euler angles about
  // Body-fixed z-y'-z'' axes by [ea(0), ea(1), ea(2)].
  Vector3d input_angles(0.5, 0.4, 0.3);
  Matrix3d bodyZYZ_angles = CalcRotationMatrixAboutZ(input_angles(0)) *
                            CalcRotationMatrixAboutY(input_angles(1)) *
                            CalcRotationMatrixAboutZ(input_angles(2));
  auto output_angles = bodyZYZ_angles.eulerAngles(2, 1, 2);
  // input_angles.isApprox(output_angles) is a valid test (rathan than
  // comparing the converted quaternions) since all the angles are between
  // 0 and PI/2.
  EXPECT_TRUE(input_angles.isApprox(output_angles));
}
class RotationConversionTest : public ::testing::Test {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 protected:
  void SetUp() override {
    setupRPYTestCases();
    setupAngleAxisTestCases();
    setupQuaternionTestCases();
    setupRotationMatrixTestCases();
  }

  void setupRPYTestCases() {
    // Set up a variety of specific tests for angles that may cause numerical
    // problems as well as a sweep of values to test general functionality.
    // Singularity issue associated with the second angle = pi/2
    // in Euler Body-fixed z-y'-x'' rotation sequence.
    // Singularity issue associated with the second angle = -pi/2
    // in Euler Body-fixed z-y'-x'' rotation sequence.
    // Singularity issue associated with the second angle close to pi/2
    // in Euler Body-fixed z-y'-x'' rotation sequence.
    // Singularity issue associated with the second angle close to -pi/2
    // in Euler Body-fixed z-y'-x'' rotation sequence.

    // pitch = pi/2
    rpy_test_cases_.push_back(Vector3d(M_PI / 4, M_PI / 2, M_PI / 3));

    // pitch = -pi/2
    rpy_test_cases_.push_back(Vector3d(M_PI / 4, -M_PI / 2, M_PI / 3));

    // pitch = 0.5*pi-eps
    rpy_test_cases_.push_back(Vector3d(
        M_PI / 4, 0.5 * M_PI - numeric_limits<double>::epsilon(), M_PI / 3));

    // pitch = 0.5*pi-1.5*eps
    rpy_test_cases_.push_back(
        Vector3d(M_PI / 4, 0.5 * M_PI - 1.5 * numeric_limits<double>::epsilon(),
                 M_PI / 3));

    // pitch = 0.5*pi-2*eps
    rpy_test_cases_.push_back(
        Vector3d(M_PI / 4, 0.5 * M_PI - 2 * numeric_limits<double>::epsilon(),
                 M_PI / 3));

    // pitch = 0.5*pi - 1E-15
    rpy_test_cases_.push_back(
        Vector3d(M_PI * 0.8, 0.5 * M_PI - 1E-15, 0.9 * M_PI));

    // pitch = -0.5*pi+eps
    rpy_test_cases_.push_back(
        Vector3d(M_PI * -0.9, -0.5 * M_PI + numeric_limits<double>::epsilon(),
                 M_PI * 0.3));

    // pitch = -0.5*pi+1.5*eps
    rpy_test_cases_.push_back(Vector3d(
        M_PI * -0.6, -0.5 * M_PI + 1.5 * numeric_limits<double>::epsilon(),
        M_PI * 0.3));

    // pitch = -0.5*pi+2*eps
    rpy_test_cases_.push_back(Vector3d(
        M_PI * -0.5, -0.5 * M_PI + 2 * numeric_limits<double>::epsilon(),
        M_PI * 0.4));

    // pitch = -0.5*pi + 1E-15
    rpy_test_cases_.push_back(
        Vector3d(M_PI * 0.9, -0.5 * M_PI + 1E-15, 0.8 * M_PI));

    // non-singular cases
    auto roll = Eigen::VectorXd::LinSpaced(Eigen::Sequential, kSweepSize,
                                           -0.99 * M_PI, M_PI);
    auto pitch = Eigen::VectorXd::LinSpaced(Eigen::Sequential, kSweepSize,
                                            -0.49 * M_PI, 0.49 * M_PI);
    auto yaw = Eigen::VectorXd::LinSpaced(Eigen::Sequential, kSweepSize,
                                          -0.99 * M_PI, M_PI);
    for (int i = 0; i < roll.size(); ++i) {
      for (int j = 0; j < pitch.size(); ++j) {
        for (int k = 0; k < yaw.size(); ++k) {
          rpy_test_cases_.push_back(Vector3d(roll(i), pitch(j), yaw(k)));
        }
      }
    }
  }

  void addAngleAxisTestCase(double angle, const Vector3d& axis) {
    angle_axis_test_cases_.push_back(AngleAxisd(angle, axis));
  }
  void setupAngleAxisTestCases() {
    // Set up a variety of specific tests for angles/axes that may cause
    // numerical problems as well as a sweep of values to test general
    // functionality.
    // Degenerate case, 0 rotation around x axis
    // Degenerate case, 0 rotation around y axis
    // Degenerate case, 0 rotation around z axis
    // Degenerate case, 0 rotation around a unit axis
    // Almost degenerate case, small positive rotation around an arbitrary axis
    // Almost degenerate case, small negative rotation around an arbitrary axis
    // Differentiation issue at 180 rotation around x axis
    // Differentiation issue at 180 rotation around y axis
    // Differentiation issue at 180 rotation around z axis
    // Differentiation issue at 180 rotation around an arbitrary unit axis
    // Differentiation issue close to 180 rotation around an arbitrary axis

    // 0 rotation around x axis
    addAngleAxisTestCase(0, Vector3d::UnitX());

    // 0 rotation around y axis
    addAngleAxisTestCase(0, Vector3d::UnitY());

    // 0 rotation around z axis
    addAngleAxisTestCase(0, Vector3d::UnitZ());

    // 0 rotation around an arbitrary axis
    Vector3d axis(0.5 * sqrt(2), 0.4 * sqrt(2), 0.3 * sqrt(2));
    addAngleAxisTestCase(0, axis);

    // epsilon rotation around an arbitrary axis
    addAngleAxisTestCase(numeric_limits<double>::epsilon(), axis);

    // 1E-10 rotation around an arbitrary axis
    addAngleAxisTestCase(1E-10, axis);

    // -epsilon rotation around an arbitary axis
    addAngleAxisTestCase(-numeric_limits<double>::epsilon(), axis);

    // -1E-10 rotation around an arbitrary axis
    addAngleAxisTestCase(-1E-10, axis);

    // 180 rotation around x axis
    addAngleAxisTestCase(M_PI, Vector3d::UnitX());

    // 180 rotation around y axis
    addAngleAxisTestCase(M_PI, Vector3d::UnitY());

    // 180 rotation around z axis
    addAngleAxisTestCase(M_PI, Vector3d::UnitZ());

    // -180 rotation around x axis
    addAngleAxisTestCase(-M_PI, Vector3d::UnitX());

    // -180 rotation around y axis
    addAngleAxisTestCase(-M_PI, Vector3d::UnitY());

    // -180 rotation around z axis
    addAngleAxisTestCase(-M_PI, Vector3d::UnitZ());

    // 180 rotation around an arbitary axis
    addAngleAxisTestCase(M_PI, axis);

    // -180 rotation around an arbitary axis
    addAngleAxisTestCase(-M_PI, axis);

    // (1-epsilon)*pi rotation around an arbitrary axis
    addAngleAxisTestCase((1 - numeric_limits<double>::epsilon()) * M_PI, axis);

    // (-1+epsilon)*pi rotation around an arbitrary axis
    addAngleAxisTestCase((-1 + numeric_limits<double>::epsilon()) * M_PI, axis);

    // (1-2*epsilon)*pi rotation around an arbitrary axis
    addAngleAxisTestCase((1 - 2 * numeric_limits<double>::epsilon()) * M_PI,
                         axis);

    // (-1+2*epsilon)*pi rotation around an arbitrary axis
    addAngleAxisTestCase((-1 + 2 * numeric_limits<double>::epsilon()) * M_PI,
                         axis);

    // (1-1E-10)*pi rotation around an arbitrary axis
    addAngleAxisTestCase((1 - 1E-10) * M_PI, axis);

    // (-1+1E-10)*pi rotation around an arbitrary axis
    addAngleAxisTestCase((-1 + 1E-10) * M_PI, axis);

    // non-singularity cases
    auto a_x = Eigen::VectorXd::LinSpaced(Eigen::Sequential, kSweepSize, -1, 1);
    auto a_y = Eigen::VectorXd::LinSpaced(Eigen::Sequential, kSweepSize, -1, 1);
    auto a_z = Eigen::VectorXd::LinSpaced(Eigen::Sequential, kSweepSize, -1, 1);
    auto a_angle = Eigen::VectorXd::LinSpaced(Eigen::Sequential, kSweepSize,
                                              -0.95 * M_PI, 0.95 * M_PI);
    for (int i = 0; i < a_x.size(); ++i) {
      for (int j = 0; j < a_y.size(); ++j) {
        for (int k = 0; k < a_z.size(); ++k) {
          Vector3d axis_ijk(a_x(i), a_y(j), a_z(k));
          if (axis_ijk.norm() > 1E-3) {
            axis_ijk.normalize();
            for (int l = 0; l < a_angle.size(); ++l) {
              addAngleAxisTestCase(a_angle(l), axis_ijk);
            }
          }
        }
      }
    }
  }

  void setupQuaternionTestCases() {
    // Set up a variety of general tests for quaternions.
    auto qw = Eigen::VectorXd::LinSpaced(Eigen::Sequential, kSweepSize, -1, 1);
    auto qx = Eigen::VectorXd::LinSpaced(Eigen::Sequential, kSweepSize, -1, 1);
    auto qy = Eigen::VectorXd::LinSpaced(Eigen::Sequential, kSweepSize, -1, 1);
    auto qz = Eigen::VectorXd::LinSpaced(Eigen::Sequential, kSweepSize, -1, 1);
    for (int i = 0; i < qw.size(); ++i) {
      for (int j = 0; j < qx.size(); ++j) {
        for (int k = 0; k < qy.size(); ++k) {
          for (int l = 0; l < qz.size(); ++l) {
            Vector4d q(qw(i), qx(j), qy(k), qz(l));
            if (q.norm() > 1E-3) {
              q.normalize();
              quaternion_test_cases_.push_back(
                  Quaterniond(q(0), q(1), q(2), q(3)));
            }
          }
        }
      }
    }
  }

  void setupRotationMatrixTestCases() {
    for (auto const& rpyi : rpy_test_cases_) {
      rotation_matrix_test_cases_.push_back(rpy2rotmat(rpyi));
    }
    for (auto const& ai : angle_axis_test_cases_) {
      rotation_matrix_test_cases_.push_back(
          axis2rotmat(EigenAngleAxisToDrakeAxisAngle(ai)));
    }
    for (auto const& qi : quaternion_test_cases_) {
      auto q = EigenQuaternionToOrderWXYZ((qi));
      auto R = quat2rotmat(q);
      rotation_matrix_test_cases_.push_back(R);
    }
  }
  std::vector<Vector3d> rpy_test_cases_;
  std::vector<AngleAxisd> angle_axis_test_cases_;
  std::vector<Quaterniond> quaternion_test_cases_;
  std::vector<Matrix3d> rotation_matrix_test_cases_;
};

TEST_F(RotationConversionTest, AxisQuat) {
  for (const auto& ai_eigen : angle_axis_test_cases_) {
    // Compute the quaternion using Eigen geometry module, compare the result
    // with axis2quat
    auto ai = EigenAngleAxisToDrakeAxisAngle(ai_eigen);
    auto q_eigen_expected = Eigen::Quaternion<double>(ai_eigen);
    auto q = axis2quat(ai);
    auto q_eigen = quat2eigenQuaternion(q);
    EXPECT_TRUE(q_eigen.isApprox(q_eigen_expected));
    // axis2quat should be the inversion of quat2axis
    auto a_expected = quat2axis(q);
    AngleAxisd a_eigen_expected(a_expected(3), a_expected.head<3>());
    EXPECT_TRUE(AreAngleAxisForSameOrientation(ai_eigen, a_eigen_expected));
  }
}

TEST_F(RotationConversionTest, AxisRotmat) {
  for (const auto& ai_eigen : angle_axis_test_cases_) {
    // Manually computes the rotation matrix from axis-angle representation,
    // using Rodrigues' rotation formula
    // https://en.wikipedia.org/wiki/Rodrigues%27_rotation_formula.
    // This is just to make sure that whatever our implementation of axis2rotmat
    // is, it outputs the right result.
    auto ai = EigenAngleAxisToDrakeAxisAngle(ai_eigen);
    auto axis_skew = VectorToSkewSymmetric(ai.head<3>());
    auto rotmat_expected = Matrix3d::Identity() + std::sin(ai(3)) * axis_skew +
                           (1.0 - std::cos(ai(3))) * axis_skew * axis_skew;
    auto rotmat = axis2rotmat(ai);
    EXPECT_TRUE(CompareMatrices(rotmat, rotmat_expected, 1E-10,
                                MatrixCompareType::absolute));

    // Compute the rotation matrix using Eigen geometry module, compare the
    // result with axis2rotmat
    auto rotmat_expected2 = ai_eigen.toRotationMatrix();
    EXPECT_TRUE(CompareMatrices(rotmat_expected2, rotmat, 1E-10,
                                MatrixCompareType::absolute));

    // axis2rotmat should be the inversion of rotmat2axis
    auto a_expected = rotmat2axis(rotmat);
    AngleAxisd a_eigen_expected(a_expected(3), a_expected.head<3>());
    EXPECT_TRUE(AreAngleAxisForSameOrientation(ai_eigen, a_eigen_expected));
  }
}

TEST_F(RotationConversionTest, AxisRPY) {
  for (const auto& ai_eigen : angle_axis_test_cases_) {
    Vector4d ai = EigenAngleAxisToDrakeAxisAngle(ai_eigen);
    Vector3d rpy = axis2rpy(ai);
    // axis2rpy should be the inversion of rpy2axis
    Vector4d a_expected = rpy2axis(rpy);
    AngleAxisd a_eigen_expected(a_expected(3), a_expected.head<3>());
    EXPECT_TRUE(AreAngleAxisForSameOrientation(ai_eigen, a_eigen_expected));
    EXPECT_TRUE(check_rpy_range(rpy));
  }
}

TEST_F(RotationConversionTest, QuatAxis) {
  for (const auto& qi_eigen : quaternion_test_cases_) {
    // Compute the angle-axis representation using Eigen geometry module,
    // compare the result with quat2axis
    Vector4d qi = EigenQuaternionToOrderWXYZ(qi_eigen);
    auto a_eigen_expected = Eigen::AngleAxis<double>(qi_eigen);

    Vector4d a = quat2axis(qi);
    auto a_eigen = Eigen::AngleAxis<double>(a(3), a.head<3>());
    EXPECT_TRUE(AreAngleAxisForSameOrientation(a_eigen, a_eigen_expected));
    // quat2axis should be the inversion of axis2quat
    Vector4d quat_expected = axis2quat(a);
    EXPECT_TRUE(AreQuaternionsForSameOrientation(qi, quat_expected));
  }
}

TEST_F(RotationConversionTest, QuatRotmat) {
  for (const auto& qi_eigen : quaternion_test_cases_) {
    // Compute the rotation matrix using Eigen geometry module, compare the
    // result with quat2rotmat
    Vector4d qi = EigenQuaternionToOrderWXYZ(qi_eigen);
    Matrix3d rotmat_expected = qi_eigen.toRotationMatrix();
    Matrix3d rotmat = quat2rotmat(qi);
    EXPECT_TRUE(CompareMatrices(rotmat_expected, rotmat, 1E-10,
                                MatrixCompareType::absolute));
    // quat2rotmat should be the inversion of rotmat2quat
    auto quat_expected = rotmat2quat(rotmat);
    EXPECT_TRUE(AreQuaternionsForSameOrientation(qi, quat_expected));
  }
}

TEST_F(RotationConversionTest, QuatRPY) {
  for (const auto& qi_eigen : quaternion_test_cases_) {
    Vector4d qi = EigenQuaternionToOrderWXYZ(qi_eigen);
    Vector3d rpy = quat2rpy(qi);
    // quat2rpy should be the inversion of rpy2quat
    Vector4d quat_expected = rpy2quat(rpy);
    EXPECT_TRUE(AreQuaternionsForSameOrientation(qi, quat_expected));
    EXPECT_TRUE(check_rpy_range(rpy));
  }
}

TEST_F(RotationConversionTest, QuatEigenQuaternion) {
  for (const auto& qi_eigen : quaternion_test_cases_) {
    Vector4d qi = EigenQuaternionToOrderWXYZ(qi_eigen);
    Quaterniond eigenQuat = quat2eigenQuaternion(qi);
    Matrix3d R_expected = quat2rotmat(qi);
    Matrix3d R_eigen = eigenQuat.matrix();
    EXPECT_TRUE(CompareMatrices(R_expected, R_eigen, 1e-6,
                                MatrixCompareType::absolute));
  }
}
TEST_F(RotationConversionTest, RotmatQuat) {
  // Compute the quaternion using Eigen geomery module, compare the result with
  // rotmat2quat
  for (const auto& Ri : rotation_matrix_test_cases_) {
    Vector4d quat = rotmat2quat(Ri);
    auto quat_expected_eigen = Quaterniond(Ri);
    Vector4d quat_expectd = EigenQuaternionToOrderWXYZ(quat_expected_eigen);
    EXPECT_TRUE(AreQuaternionsForSameOrientation(quat, quat_expectd));
    // rotmat2quat should be the inversion of quat2rotmat
    Matrix3d rotmat_expected = quat2rotmat(quat);
    EXPECT_TRUE(Ri.isApprox(rotmat_expected));
  }
}

TEST_F(RotationConversionTest, RotmatAxis) {
  // Compute angle-axis using Eigen geometry module, compare the result with
  // rotmat2axis
  for (const auto& Ri : rotation_matrix_test_cases_) {
    auto a_eigen_expected = AngleAxisd(Ri);
    Vector4d a = rotmat2axis(Ri);
    auto a_eigen = AngleAxisd(a(3), a.head<3>());
    EXPECT_TRUE(AreAngleAxisForSameOrientation(a_eigen, a_eigen_expected));
    // rotmat2axis should be the inversion of axis2rotmat
    auto rotmat_expected = axis2rotmat(a);
    EXPECT_TRUE(Ri.isApprox(rotmat_expected));
  }
}

TEST_F(RotationConversionTest, RotmatRPY) {
  for (const auto& Ri : rotation_matrix_test_cases_) {
    Vector3d rpy = rotmat2rpy(Ri);
    // rotmat2rpy should be the inversion of rpy2rotmat
    Matrix3d rotmat_expected = rpy2rotmat(rpy);
    EXPECT_TRUE(Ri.isApprox(rotmat_expected, 1E-10));
    EXPECT_TRUE(check_rpy_range(rpy));
  }
}

TEST_F(RotationConversionTest, RPYRotmat) {
  // Compute the rotation matrix by rotz(rpy(2))*roty(rpy(1))*rotx(rpy(0)),
  // then compare the result with rpy2rotmat
  for (const auto& rpyi : rpy_test_cases_) {
    Quaterniond rotation_expected =
        Eigen::AngleAxisd(rpyi(2), Vector3d::UnitZ()) *
        Eigen::AngleAxisd(rpyi(1), Vector3d::UnitY()) *
        Eigen::AngleAxisd(rpyi(0), Vector3d::UnitX());
    Matrix3d rotmat = rpy2rotmat(rpyi);
    EXPECT_TRUE(CompareMatrices(rotmat, rotation_expected.toRotationMatrix(),
                                1E-10, MatrixCompareType::absolute));
    // rpy2rotmat should be the inversion of rotmat2rpy
    Vector3d rpy_expected = rotmat2rpy(rotmat);
    EXPECT_TRUE(AreRollPitchYawForSameOrientation(rpyi, rpy_expected));
  }
}

TEST_F(RotationConversionTest, RPYAxis) {
  // Compute the angle-axis representation using Eigen's geometry module,
  // compare the result with rpy2axis
  for (const auto& rpyi : rpy_test_cases_) {
    Quaterniond quaternion_expected =
        Eigen::AngleAxisd(rpyi(2), Vector3d::UnitZ()) *
        Eigen::AngleAxisd(rpyi(1), Vector3d::UnitY()) *
        Eigen::AngleAxisd(rpyi(0), Vector3d::UnitX());
    auto eigenAngleAxis_expected =
        Eigen::AngleAxis<double>(quaternion_expected);

    Vector4d drakeAxisAngle_test = rpy2axis(rpyi);
    AngleAxisd eigenAngleAxis_test = axisToEigenAngleAxis(drakeAxisAngle_test);
    EXPECT_TRUE(AreAngleAxisForSameOrientation(eigenAngleAxis_test,
                                               eigenAngleAxis_expected));

    // rpy2axis should be the inversion of axis2rpy
    Vector3d rpy_test_drake = axis2rpy(drakeAxisAngle_test);
    EXPECT_TRUE(AreRollPitchYawForSameOrientation(rpyi, rpy_test_drake));
  }
}

// Verifies the correctness of the method drake::math::rpy2quat() by comparing
// its output to a quaternion obtained using the composition of
// Eigen::AngleAxisd transformations in the method
// BodyZYXAnglesToEigenQuaternion() local in this file.
TEST_F(RotationConversionTest, RPYQuat) {
  // Compute the quaternion representation using Eigen's geometry model,
  // compare the result with rpy2quat
  for (const auto& rpyi : rpy_test_cases_) {
    Vector3d bodyZYX_angles(rpyi(2), rpyi(1), rpyi(0));
    auto quat_eigen_expected = BodyZYXAnglesToEigenQuaternion(bodyZYX_angles);
    auto quat_expected = EigenQuaternionToOrderWXYZ(quat_eigen_expected);
    auto quat = rpy2quat(rpyi);
    EXPECT_TRUE(AreQuaternionsForSameOrientation(quat, quat_expected));
    // rpy2quat should be the inversion of quat2rpy
    auto rpy_expected = quat2rpy(quat);
    EXPECT_TRUE(AreRollPitchYawForSameOrientation(rpyi, rpy_expected));
    EXPECT_TRUE(
        AreRollPitchYawForSameOrientation(rpyi, rotmat2rpy(rpy2rotmat(rpyi))));
  }
}

// Verifies the correctness of the method
// drake::math::RollPitchYawToQuaternion() by comparing
// its output to a quaternion obtained using the composition of
// Eigen::AngleAxisd transformations in the method
// BodyZYXAnglesToEigenQuaternion() local in this file.
TEST_F(RotationConversionTest, RollPitchYawToQuaternion) {
  // Compute the quaternion representation using Eigen's geometry model,
  // compare the result with rpy2quat
  for (const auto& rpyi : rpy_test_cases_) {
    Vector3d bodyZYX_angles(rpyi(2), rpyi(1), rpyi(0));
    auto quat_expected = BodyZYXAnglesToEigenQuaternion(bodyZYX_angles);
    auto quat = RollPitchYawToQuaternion(rpyi);
    EXPECT_TRUE(
        quat.isApprox(quat_expected, Eigen::NumTraits<double>::epsilon()));
  }
}

}  // namespace
}  // namespace math
}  // namespace drake
