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

const double kEpsilon = numeric_limits<double>::epsilon();
const double kTolerance = 1.0E-12;

GTEST_TEST(EigenEulerAngleTest, MakeXYZRotation) {
  // Verify MakeXRotation(theta), MakeYRotation(theta), MakeZRotation(theta) is
  // the same as AngleAxis equivalents.
  const double theta = 0.1234567;  // Arbitrary angle.
  const RotationMatrixd Rx(RotationMatrixd::MakeXRotation(theta));
  const RotationMatrixd Ry(RotationMatrixd::MakeYRotation(theta));
  const RotationMatrixd Rz(RotationMatrixd::MakeZRotation(theta));
  const Quaterniond qx(Eigen::AngleAxisd(theta, Vector3d::UnitX()));
  const Quaterniond qy(Eigen::AngleAxisd(theta, Vector3d::UnitY()));
  const Quaterniond qz(Eigen::AngleAxisd(theta, Vector3d::UnitZ()));
  const double tolerance = 32 * kEpsilon;
  EXPECT_TRUE(Rx.IsNearlyEqualTo(RotationMatrixd(qx), tolerance));
  EXPECT_TRUE(Ry.IsNearlyEqualTo(RotationMatrixd(qy), tolerance));
  EXPECT_TRUE(Rz.IsNearlyEqualTo(RotationMatrixd(qz), tolerance));
}

GTEST_TEST(EigenEulerAngleTest, BodyXYZ) {
  // Verify ea = Eigen::eulerAngles(0, 1, 2) returns Euler angles about
  // Body-fixed x-y'-z'' axes by [ea(0), ea(1), ea(2)].
  const Vector3d input_angles(0.5, 0.4, 0.3);
  const Matrix3d bodyXYZ_rotmat =
      (RotationMatrix<double>::MakeXRotation(input_angles(0)) *
       RotationMatrix<double>::MakeYRotation(input_angles(1)) *
       RotationMatrix<double>::MakeZRotation(input_angles(2))).matrix();
  const Vector3d output_angles = bodyXYZ_rotmat.eulerAngles(0, 1, 2);
  // input_angles.isApprox(output_angles) is a valid test (rathan than
  // comparing the converted quaternions) since all the angles are between
  // 0 and PI/2.
  EXPECT_TRUE(input_angles.isApprox(output_angles));
}

GTEST_TEST(EigenEulerAngleTest, SpaceXYZ) {
  // Verify ea = Eigen::eulerAngles(2, 1, 0) returns Euler angles about
  // Body-fixed z-y'-x'' axes by [ea(0), ea(1), ea(2)].
  const Vector3d input_angles(0.5, 0.4, 0.3);
  const Matrix3d spaceXYZ_rotmat =
      (RotationMatrix<double>::MakeZRotation(input_angles(0)) *
       RotationMatrix<double>::MakeYRotation(input_angles(1)) *
       RotationMatrix<double>::MakeXRotation(input_angles(2))).matrix();
  const Vector3d output_angles = spaceXYZ_rotmat.eulerAngles(2, 1, 0);
  // input_angles.isApprox(output_angles) is a valid test (rathan than
  // comparing the converted quaternions) since all the angles are between
  // 0 and PI/2.
  EXPECT_TRUE(input_angles.isApprox(output_angles));
}

GTEST_TEST(EigenEulerAngleTest, BodyZYZ) {
  // Verify ea = Eigen::eulerAngles(2, 1, 0) returns Euler angles about
  // Body-fixed z-y'-z'' axes by [ea(0), ea(1), ea(2)].
  const Vector3d input_angles(0.5, 0.4, 0.3);
  const Matrix3d bodyZYZ_angles =
      (RotationMatrix<double>::MakeZRotation(input_angles(0)) *
       RotationMatrix<double>::MakeYRotation(input_angles(1)) *
       RotationMatrix<double>::MakeZRotation(input_angles(2))).matrix();
  const Vector3d output_angles = bodyZYZ_angles.eulerAngles(2, 1, 2);
  // input_angles.isApprox(output_angles) is a valid test (rathan than
  // comparing the converted quaternions) since all the angles are between
  // 0 and PI/2.
  EXPECT_TRUE(input_angles.isApprox(output_angles));
}

class RotationConversionTest : public ::testing::Test {
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
    const RollPitchYaw<double> pitch_half_pi(M_PI / 4, M_PI / 2, M_PI / 3);
    rpy_test_cases_.push_back(pitch_half_pi);

    // pitch = -pi/2
    const RollPitchYaw<double> pitch_neg_half_pi(M_PI / 4, -M_PI / 2, M_PI / 3);
    rpy_test_cases_.push_back(pitch_neg_half_pi);

    // pitch = 0.5*pi-eps
    const RollPitchYaw<double> pitch_near_half_piA(
        M_PI / 4, 0.5 * M_PI - kEpsilon, M_PI / 3);
    rpy_test_cases_.push_back(pitch_near_half_piA);

    // pitch = 0.5*pi-1.5*eps
    const RollPitchYaw<double> pitch_near_half_piB(
        M_PI / 4, 0.5 * M_PI - 1.5 * kEpsilon, M_PI / 3);
    rpy_test_cases_.push_back(pitch_near_half_piB);

    // pitch = 0.5*pi-2*eps
    const RollPitchYaw<double> pitch_near_half_piC(
        M_PI / 4, 0.5 * M_PI - 2 * kEpsilon, M_PI / 3);
    rpy_test_cases_.push_back(pitch_near_half_piC);

    // pitch = 0.5*pi - 1E-15
    const RollPitchYaw<double> pitch_near_half_piD(
        M_PI * 0.8, 0.5 * M_PI - 1E-15, 0.9 * M_PI);
    rpy_test_cases_.push_back(pitch_near_half_piD);

    // pitch = -0.5*pi+eps
    const RollPitchYaw<double> pitch_near_neg_half_piA(
        M_PI * -0.9, -0.5 * M_PI + kEpsilon, M_PI * 0.3);
    rpy_test_cases_.push_back(pitch_near_neg_half_piA);

    // pitch = -0.5*pi+1.5*eps
    const RollPitchYaw<double> pitch_near_neg_half_piB(
        M_PI * -0.6, -0.5 * M_PI + 1.5 * kEpsilon, M_PI * 0.3);
    rpy_test_cases_.push_back(pitch_near_neg_half_piB);

    // pitch = -0.5*pi+2*eps
    const RollPitchYaw<double> pitch_near_neg_half_piC(
        M_PI * -0.5, -0.5 * M_PI + 2 * kEpsilon, M_PI * 0.4);
    rpy_test_cases_.push_back(pitch_near_neg_half_piC);

    // pitch = -0.5*pi + 1E-15
    const RollPitchYaw<double> pitch_near_neg_half_piD(
        M_PI * 0.9, -0.5 * M_PI + 1E-15, 0.8 * M_PI);
    rpy_test_cases_.push_back(pitch_near_neg_half_piD);

    // non-singular cases
    auto roll = Eigen::VectorXd::LinSpaced(kSweepSize,
                                           -0.99 * M_PI, M_PI);
    auto pitch = Eigen::VectorXd::LinSpaced(kSweepSize,
                                            -0.49 * M_PI, 0.49 * M_PI);
    auto yaw = Eigen::VectorXd::LinSpaced(kSweepSize,
                                          -0.99 * M_PI, M_PI);
    for (int i = 0; i < roll.size(); ++i) {
      for (int j = 0; j < pitch.size(); ++j) {
        for (int k = 0; k < yaw.size(); ++k) {
          const RollPitchYaw<double> rpy(roll(i), pitch(j), yaw(k));
          rpy_test_cases_.push_back(rpy);
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
    addAngleAxisTestCase(kEpsilon, axis);

    // 1E-10 rotation around an arbitrary axis
    addAngleAxisTestCase(1E-10, axis);

    // -epsilon rotation around an arbitrary axis
    addAngleAxisTestCase(-kEpsilon, axis);

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

    // 180 rotation around an arbitrary axis
    addAngleAxisTestCase(M_PI, axis);

    // -180 rotation around an arbitrary axis
    addAngleAxisTestCase(-M_PI, axis);

    // (1-epsilon)*pi rotation around an arbitrary axis
    addAngleAxisTestCase((1 - kEpsilon) * M_PI, axis);

    // (-1+epsilon)*pi rotation around an arbitrary axis
    addAngleAxisTestCase((-1 + kEpsilon) * M_PI, axis);

    // (1-2*epsilon)*pi rotation around an arbitrary axis
    addAngleAxisTestCase((1 - 2 * kEpsilon) * M_PI, axis);

    // (-1+2*epsilon)*pi rotation around an arbitrary axis
    addAngleAxisTestCase((-1 + 2 * kEpsilon) * M_PI, axis);

    // (1-1E-10)*pi rotation around an arbitrary axis
    addAngleAxisTestCase((1 - 1E-10) * M_PI, axis);

    // (-1+1E-10)*pi rotation around an arbitrary axis
    addAngleAxisTestCase((-1 + 1E-10) * M_PI, axis);

    // non-singularity cases
    auto a_x = Eigen::VectorXd::LinSpaced(kSweepSize, -1, 1);
    auto a_y = Eigen::VectorXd::LinSpaced(kSweepSize, -1, 1);
    auto a_z = Eigen::VectorXd::LinSpaced(kSweepSize, -1, 1);
    auto a_angle = Eigen::VectorXd::LinSpaced(kSweepSize,
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
    auto qw = Eigen::VectorXd::LinSpaced(kSweepSize, -1, 1);
    auto qx = Eigen::VectorXd::LinSpaced(kSweepSize, -1, 1);
    auto qy = Eigen::VectorXd::LinSpaced(kSweepSize, -1, 1);
    auto qz = Eigen::VectorXd::LinSpaced(kSweepSize, -1, 1);
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
    for (const RollPitchYaw<double>& rpyi : rpy_test_cases_) {
      const RotationMatrix<double> Ri(rpyi);
      rotation_matrix_test_cases_.push_back(Ri);
    }
    for (const Eigen::AngleAxisd& ai : angle_axis_test_cases_) {
      const RotationMatrix<double> Ri(ai);
      rotation_matrix_test_cases_.push_back(Ri);
    }
    for (const Quaterniond& qi : quaternion_test_cases_) {
      const RotationMatrix<double> Ri(qi);
      rotation_matrix_test_cases_.push_back(Ri);
    }
  }
  std::vector<RollPitchYaw<double>> rpy_test_cases_;
  std::vector<AngleAxisd> angle_axis_test_cases_;
  std::vector<Quaterniond> quaternion_test_cases_;
  std::vector<RotationMatrix<double>> rotation_matrix_test_cases_;
};

TEST_F(RotationConversionTest, quaternionToRotationMatrixTest) {
  for (const Quaterniond& qi : quaternion_test_cases_) {
    // Compute the rotation matrix using Eigen geometry module, compare the
    // result with RotationMatrix(quaternion).
    const Matrix3d rotmat_expected = qi.toRotationMatrix();
    const Matrix3d rotmat = RotationMatrix<double>(qi).matrix();
    EXPECT_TRUE(CompareMatrices(rotmat_expected, rotmat, 1E-10,
                                MatrixCompareType::absolute));
    // RotationMatrix(quaternion) is inverse of RotationMatrix::ToQuaternion().
    const Eigen::Quaterniond quat_expected =
        RotationMatrix<double>::ToQuaternion(rotmat);
    EXPECT_TRUE(
        AreQuaternionsEqualForOrientation(qi, quat_expected, kTolerance));
  }
}

TEST_F(RotationConversionTest, QuatRPY) {
  for (const Quaterniond& qi : quaternion_test_cases_) {
    const RollPitchYaw<double> rpy(qi);
    const Eigen::Quaterniond q_expected = rpy.ToQuaternion();
    // Test RollPitchYaw::ToQuaternion() is inverse of RollPitchYaw(quaternion).
    EXPECT_TRUE(AreQuaternionsEqualForOrientation(qi, q_expected, kTolerance));
    EXPECT_TRUE(rpy.IsRollPitchYawInCanonicalRange());
  }
}

TEST_F(RotationConversionTest, RotmatQuat) {
  // Compare Eigen's rotation matrix to quaternion result with the result from
  // RotationMatrix::ToQuaternion().
  for (const RotationMatrix<double>& Ri : rotation_matrix_test_cases_) {
    const Eigen::Quaterniond quat_drake = Ri.ToQuaternion();
    const Eigen::Quaterniond quat_eigen = Quaterniond(Ri.matrix());
    EXPECT_TRUE(
        AreQuaternionsEqualForOrientation(quat_drake, quat_eigen, kTolerance));
    // Ensure the calculated quaternion produces the same rotation matrix.
    // This test accuracy to near machine precision and uses a tolerance of
    // 32 * kEpsilon (allows for 5 of the 53 mantissa bits to be inaccurate).
    // This 5-bit estimate seems to be a reasonably tight bound which
    // nevertheless passes a representative sampling of compilers and platforms.
    const RotationMatrix<double> rotmat(quat_drake);
    EXPECT_TRUE(Ri.IsNearlyEqualTo(rotmat, 32 * kEpsilon));
  }
}

TEST_F(RotationConversionTest, rotmat2rpyTest) {
  for (const RotationMatrix<double>& Ri : rotation_matrix_test_cases_) {
    const RollPitchYaw<double> rpy(Ri);
    const RotationMatrix<double> rotmat_expected(rpy);
    // RollPitchYaw(RotationMatrix) is inverse of RotationMatrix(RollPitchYaw).
    EXPECT_TRUE(Ri.IsNearlyEqualTo(rotmat_expected, 256 * kEpsilon));
    EXPECT_TRUE(rpy.IsRollPitchYawInCanonicalRange());
  }
}

TEST_F(RotationConversionTest, rpy2rotmatTest) {
  for (const RollPitchYaw<double>& rpyi : rpy_test_cases_) {
    const double roll = rpyi.roll_angle();
    const double pitch = rpyi.pitch_angle();
    const double yaw = rpyi.yaw_angle();
    const Quaterniond q = Eigen::AngleAxisd(yaw, Vector3d::UnitZ()) *
                          Eigen::AngleAxisd(pitch, Vector3d::UnitY()) *
                          Eigen::AngleAxisd(roll, Vector3d::UnitX());
    const RotationMatrix<double> R_from_quaternion(q);

    // Compute rotation matrix by rotz(rpy(2))*roty(rpy(1))*rotx(rpy(0)),
    // then compare the result with RotationMatrix(RollPitchYaw).
    const RotationMatrix<double> R_from_rpy(rpyi);
    EXPECT_TRUE(
        R_from_rpy.IsNearlyEqualTo(R_from_quaternion, 512 * kEpsilon));

    // RollPitchYaw(RotationMatrix) is inverse of RotationMatrix(RollPitchYaw).
    const RollPitchYaw<double> rpy_expected(R_from_rpy);
    EXPECT_TRUE(rpyi.IsNearlySameOrientation(rpy_expected, kTolerance));
  }
}

TEST_F(RotationConversionTest, rpy2QuatTest) {
  for (const RollPitchYaw<double>& rpyi : rpy_test_cases_) {
    const Eigen::Quaterniond q = rpyi.ToQuaternion();
    // Verify rpyi.ToQuaternion() is inverse of RollPitchYaw(Quaternion).
    const RollPitchYaw<double> rpy_expected(q);
    EXPECT_TRUE(rpyi.IsNearlySameOrientation(rpy_expected, 512 * kEpsilon));
  }
}

}  // namespace
}  // namespace math
}  // namespace drake

