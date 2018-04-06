/// @file
/// Tests that rotation conversion functions are inverses.

#include <cmath>
#include <iostream>

#include <Eigen/Dense>
#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
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

bool AreQuaternionsForSameOrientation(const Vector4d& q1, const Vector4d& q2,
                                      double precision = 1E-12) {
  // The same orientation is described by both a quaternion and the negative of
  // that quaternion.
  return q1.isApprox(q2, precision) || q1.isApprox(-q2, precision);
}

/// Returns a %Quaternion associated with a Space-fixed (extrinsic) X-Y-Z
/// rotation by "roll-pitch-yaw" angles `[r, p, y]`.
/// @param[in] rpy radian measures of "roll-pitch-yaw" angles.
template <typename T>
Eigen::Quaternion<T> SpaceXYZAnglesToEigenQuaternion(const Vector3<T> rpy) {
  using std::cos;
  using std::sin;
  const T c0 = cos(rpy(0)/2), s0 = sin(rpy(0)/2);
  const T c1 = cos(rpy(1)/2), s1 = sin(rpy(1)/2);
  const T c2 = cos(rpy(2)/2), s2 = sin(rpy(2)/2);

  const T w = c0 * c1 * c2 + s0 * s1 * s2;
  const T x = s0 * c1 * c2 - c0 * s1 * s2;
  const T y = c0 * s1 * c2 + s0 * c1 * s2;
  const T z = c0 * c1 * s2 - s0 * s1 * c2;
  return Eigen::Quaternion<T>(w, x, y, z);
}

bool AreRollPitchYawForSameOrientation(const Vector3d& rpy1,
                                       const Vector3d& rpy2) {
  // Note: When pitch is close to PI/2 or -PI/2, derivative calculations for
  // Euler angle can encounter numerical problems.  However, although values
  // of angles may "jump around" (hence, difficult derivatives), the angles'
  // values should be accurately reproduced.
  const double precision = 1E-13;
  const Eigen::Quaterniond q1 = SpaceXYZAnglesToEigenQuaternion(rpy1);
  const Eigen::Quaterniond q2 = SpaceXYZAnglesToEigenQuaternion(rpy2);
  return AreQuaternionsForSameOrientation(EigenQuaternionToOrderWXYZ(q1),
                                          EigenQuaternionToOrderWXYZ(q2),
                                          precision);
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

GTEST_TEST(EigenEulerAngleTest, SpaceXYZ) {
  // Verify ea = Eigen::eulerAngles(2, 1, 0) returns Euler angles about
  // Body-fixed z-y'-x'' axes by [ea(0), ea(1), ea(2)].
  const Vector3d input_angles(0.5, 0.4, 0.3);
  const Matrix3d spaceXYZ_rotmat = CalcRotationMatrixAboutZ(input_angles(0)) *
                                   CalcRotationMatrixAboutY(input_angles(1)) *
                                   CalcRotationMatrixAboutX(input_angles(2));
  auto output_angles = spaceXYZ_rotmat.eulerAngles(2, 1, 0);
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
    SetupRPYTestCases();
    SetupAngleAxisTestCases();
    SetupQuaternionTestCases();
    SetupRotationMatrixTestCases();
  }

  void SetupRPYTestCases() {
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

  void SetupAngleAxisTestCases() {
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

  void SetupQuaternionTestCases() {
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

  void SetupRotationMatrixTestCases() {
    for (const Vector3d& rpyi : rpy_test_cases_) {
      rotation_matrix_test_cases_.push_back(rpy2rotmat(rpyi));
    }
    for (const Eigen::AngleAxisd& ai : angle_axis_test_cases_) {
      const RotationMatrix<double> Ri(ai);
      rotation_matrix_test_cases_.push_back(Ri.matrix());
    }
    for (const Quaterniond& qi : quaternion_test_cases_) {
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

TEST_F(RotationConversionTest, QuatRotmat) {
  for (const Quaterniond& qi_eigen : quaternion_test_cases_) {
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
  for (const Quaterniond& qi_eigen : quaternion_test_cases_) {
    const Vector3d rpy = QuaternionToSpaceXYZ(qi_eigen);
    // rpy2quat should be the inversion of QuaternionToSpaceXYZ().
    const Vector4d quat_expected = rpy2quat(rpy);
    const Vector4d qi = EigenQuaternionToOrderWXYZ(qi_eigen);
    EXPECT_TRUE(AreQuaternionsForSameOrientation(qi, quat_expected));
    EXPECT_TRUE(check_rpy_range(rpy));
  }
}

TEST_F(RotationConversionTest, QuatEigenQuaternion) {
  for (const Quaterniond& qi_eigen : quaternion_test_cases_) {
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
  for (const Vector3d& rpyi : rpy_test_cases_) {
    const Quaterniond q = Eigen::AngleAxisd(rpyi(2), Vector3d::UnitZ()) *
                          Eigen::AngleAxisd(rpyi(1), Vector3d::UnitY()) *
                          Eigen::AngleAxisd(rpyi(0), Vector3d::UnitX());
    Matrix3d rotmat = rpy2rotmat(rpyi);
    EXPECT_TRUE(CompareMatrices(rotmat, q.toRotationMatrix(),
                                1E-10, MatrixCompareType::absolute));
    // rpy2rotmat should be the inversion of rotmat2rpy
    Vector3d rpy_expected = rotmat2rpy(rotmat);
    EXPECT_TRUE(AreRollPitchYawForSameOrientation(rpyi, rpy_expected));
  }
}

// Verifies the correctness of drake::math::QuaternionToSpaceXYZ() by comparing
// its output to a quaternion obtained from SpaceXYZAnglesToEigenQuaternion().
TEST_F(RotationConversionTest, RPYQuat) {
  for (const Vector3d& rpyi : rpy_test_cases_) {
    const Vector3d spaceXYZ_angles(rpyi(0), rpyi(1), rpyi(2));
    Eigen::Quaterniond q = SpaceXYZAnglesToEigenQuaternion(spaceXYZ_angles);
    const Vector4d quat = rpy2quat(rpyi);
    EXPECT_TRUE(AreQuaternionsForSameOrientation(
                quat, EigenQuaternionToOrderWXYZ(q)));
    // QuaternionToSpaceXYZ() should be the inversion of rpy2quat.
    const Vector3d rpy_expected = QuaternionToSpaceXYZ(q);
    EXPECT_TRUE(AreRollPitchYawForSameOrientation(rpyi, rpy_expected));
    EXPECT_TRUE(
        AreRollPitchYawForSameOrientation(rpyi, rotmat2rpy(rpy2rotmat(rpyi))));
  }
}

// Verifies the correctness of the method
// drake::math::RollPitchYawToQuaternion() by comparing its output to a
// quaternion obtained in the local function SpaceXYZAnglesToEigenQuaternion().
TEST_F(RotationConversionTest, RollPitchYawToQuaternion) {
  // Compute the quaternion representation using Eigen's geometry model,
  // compare the result with rpy2quat
  for (const Vector3d& rpyi : rpy_test_cases_) {
    const Vector3d spaceXYZ_angles(rpyi(0), rpyi(1), rpyi(2));
    auto quat_expected = SpaceXYZAnglesToEigenQuaternion(spaceXYZ_angles);
    auto quat = RollPitchYawToQuaternion(rpyi);
    EXPECT_TRUE(
        quat.isApprox(quat_expected, Eigen::NumTraits<double>::epsilon()));
  }
}

}  // namespace
}  // namespace math
}  // namespace drake
