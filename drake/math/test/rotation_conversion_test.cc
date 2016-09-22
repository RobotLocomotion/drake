/// @file
/// Tests that rotation conversion functions are inverses.

#include <cmath>
#include <iostream>

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
using std::numeric_limits;

namespace drake {
namespace math {
namespace {
Vector4d eigenQuaterniontoQuat(const Quaterniond& q) {
  return Vector4d(q.w(), q.x(), q.y(), q.z());
}

bool check_rpy_range(const Vector3d &rpy) {
  return rpy(0) <= M_PI && rpy(0) >= -M_PI && rpy(1) <= M_PI/2 && rpy(1) >= -M_PI/2 && rpy(2) <= M_PI && rpy(2) >= -M_PI;
}
// Compare the equivalent rotation matrix.
// Note that we are not comparing the axis-angle directly. This is because the
// axis-angle has singularities around 0 degree rotation and 180 degree rotation
// So two axis-angles that are slightly different when the angle is close to 0,
// their equivalent rotation matrices are almost the same
bool compareAngleAxis(const AngleAxisd& a1, const AngleAxisd& a2) {
  return a1.toRotationMatrix().isApprox(a2.toRotationMatrix(), 1E-10);
}

bool compareQuaternion(const Vector4d& q1, const Vector4d& q2) {
  return q1.isApprox(q2,1E-10) | q1.isApprox(-q2, 1E-10);
}
Quaterniond eulerToQuaternion(const Vector3d euler) {
  // Compute the quaterion for euler angle using intrinsic z-y'-x''
  return AngleAxisd(euler(0), Vector3d::UnitZ()) *
      AngleAxisd(euler(1), Vector3d::UnitY()) *
      AngleAxisd(euler(2), Vector3d::UnitX());
}

bool compareEulerAngles(const Vector3d& euler_angles1,
                        const Vector3d& euler_angles2) {
  auto q1 = eulerToQuaternion(euler_angles1);
  auto q2 = eulerToQuaternion(euler_angles2);
  return compareQuaternion(eigenQuaterniontoQuat(q1), eigenQuaterniontoQuat(q2));
}

bool compareRollPitchYaw(const Vector3d &rpy1, const Vector3d &rpy2) {
  Vector3d euler_angles1(rpy1(2), rpy1(1), rpy1(0));
  Vector3d euler_angles2(rpy2(2), rpy2(1), rpy2(0));
  return compareEulerAngles(euler_angles1, euler_angles2);
}

Vector4d eigenAxisToAxis(const AngleAxisd& a) {
  Vector4d ret;
  ret << a.axis(), a.angle();
  return ret;
}



Matrix3d rotz(double a) {
  Matrix3d ret;
  ret << cos(a), -sin(a), 0,
      sin(a), cos(a) , 0,
      0     , 0      , 1;
  return ret;
}

Matrix3d roty(double b) {
  Matrix3d ret;
  ret << cos(b) , 0, sin(b),
      0      , 1, 0      ,
      -sin(b), 0, cos(b);
  return ret;
}

Matrix3d rotx(double c) {
  Matrix3d ret;
  ret << 1, 0,      0,
      0, cos(c), -sin(c),
      0, sin(c), cos(c);
  return ret;
}
GTEST_TEST(EigenEulerAngleTest, BodyXYZ) {
  // Verify ea = Eigen::eulerAngles(0, 1, 2) returns Euler angles about
  // Body-fixed x-y'-z'' axes by [ea(0), ea(1), ea(2)]
  Vector3d ea(0.5, 0.4, 0.3);
  Matrix3d rotmat = rotx(ea(0)) * roty(ea(1)) * rotz(ea(2));
  auto ea_expected = rotmat.eulerAngles(0, 1, 2);
  EXPECT_TRUE(ea.isApprox(ea_expected));
}

GTEST_TEST(EigenEulerAngleTest, BodyZYX) {
  // Verify ea = Eigen::eulerAngles(2, 1, 0) returns Euler angles about
  // Body-fixed z-y'-x'' axes by [ea(0), ea(1), ea(2)]
  Vector3d ea(0.5, 0.4, 0.3);
  Matrix3d rotmat = rotz(ea(0)) * roty(ea(1)) * rotx(ea(2));
  auto ea_expected = rotmat.eulerAngles(2, 1, 0);
  EXPECT_TRUE(ea.isApprox(ea_expected));
}

GTEST_TEST(EigenEulerAngleTest, BodyZYZ) {
  // Verify ea = Eigen::eulerAngles(2, 1, 0) returns Euler angles about
  // Body-fixed z-y'-z'' axes by [ea(0), ea(1), ea(2)]
  Vector3d ea(0.5, 0.4, 0.3);
  Matrix3d rotmat = rotz(ea(0)) * roty(ea(1)) * rotz(ea(2));
  auto ea_expected = rotmat.eulerAngles(2, 1, 2);
  EXPECT_TRUE(ea.isApprox(ea_expected));
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
    // We will test the following cases
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
    // Singularity issue associated with the second angle = pi/2
    // in Euler Body-fixed z-y'-x'' rotation sequence
    // Singularity issue associated with the second angle = -pi/2
    // in Euler Body-fixed z-y'-x'' rotation sequence
    // Singularity issue associated with the second angle close to pi/2
    // in Euler Body-fixed z-y'-x'' rotation sequence
    // Singularity issue associated with the second angle close to -pi/2
    // in Euler Body-fixed z-y'-x'' rotation sequence
    // normal non-degenerate non-singular cases.

    /*// 0 rotation around x axis
    test_orientation_.push_back(AngleAxisd(0, Vector3d::UnitX()));

    // 0 rotation around y axis
    test_orientation_.push_back(AngleAxisd(0, Vector3d::UnitY()));

    // 0 rotation around z axis
    test_orientation_.push_back(AngleAxisd(0, Vector3d::UnitZ()));

    // 0 rotation around a unit axis
    Vector3d axis(0.5 * sqrt(2), 0.4 * sqrt(2), 0.3 * sqrt(2));
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
    test_orientation_.push_back(AngleAxisd((1 - 1E-6) * M_PI, axis));

    // pitch = pi/2
    AngleAxisd a1(AngleAxisd(M_PI / 3, Vector3d::UnitZ()) *
                  AngleAxisd(M_PI / 2, Vector3d::UnitY()) *
                  AngleAxisd(M_PI / 4, Vector3d::UnitX()));
    test_orientation_.push_back(a1);

    // pitch close to pi/2
    AngleAxisd a2(AngleAxisd(M_PI / 3, Vector3d::UnitZ()) *
                  AngleAxisd(M_PI / 2 - 1E-6 * M_PI, Vector3d::UnitY()) *
                  AngleAxisd(M_PI / 4, Vector3d::UnitX()));
    test_orientation_.push_back(a2);

    // pitch = -pi/2
    AngleAxisd a4(AngleAxisd(M_PI / 3, Vector3d::UnitZ()) *
        AngleAxisd(-M_PI / 2, Vector3d::UnitY()) *
        AngleAxisd(M_PI / 4, Vector3d::UnitX()));
    test_orientation_.push_back(a4);

    // pitch close to -pi/2
    AngleAxisd a5(AngleAxisd(M_PI / 3, Vector3d::UnitZ()) *
        AngleAxisd(-M_PI / 2 + 1E-6 * M_PI, Vector3d::UnitY()) *
        AngleAxisd(M_PI / 4, Vector3d::UnitX()));
    test_orientation_.push_back(a5);*/

    // non-degenerate case

    /*// non-degenerate case for angle-axis representation
    auto a_x = Eigen::VectorXd::LinSpaced(Eigen::Sequential, 10, -1, 1);
    auto a_y = Eigen::VectorXd::LinSpaced(Eigen::Sequential, 10, -1, 1);
    auto a_z = Eigen::VectorXd::LinSpaced(Eigen::Sequential, 10, -1, 1);
    auto a_angle = Eigen::VectorXd::LinSpaced(Eigen::Sequential, 10, -M_PI+1E-6, M_PI);
    for(int i = 0; i < a_x.size(); i++) {
      for (int j = 0; j < a_y.size(); ++j) {
        for (int k = 0; k < a_z.size(); ++k) {
          for (int l = 0; l < a_angle.size(); ++l) {
            Vector3d a_axis(a_x(i), a_y(j), a_z(k));
            if(std::abs(a_axis.norm()) > 1E-3) {
              a_axis.normalize();
              test_orientation_.push_back(AngleAxisd(a_angle(l), a_axis));
            }
          }
        }
      }
    }*/

    /*// non-degenerate case for quaternion representation
    auto q_w = Eigen::VectorXd::LinSpaced(Eigen::Sequential, 10, -1, 1);
    auto q_x = Eigen::VectorXd::LinSpaced(Eigen::Sequential, 10, -1, 1);
    auto q_y = Eigen::VectorXd::LinSpaced(Eigen::Sequential, 10, -1, 1);
    auto q_z = Eigen::VectorXd::LinSpaced(Eigen::Sequential, 10, -1, 1);
    for(int i = 0; i< q_w.size(); ++i) {
      for (int j = 0; j < q_x.size(); ++j) {
        for (int k = 0; k < q_y.size(); ++k) {
          for (int l = 0; l < q_z.size(); ++l) {
            Vector4d q(q_w(i), q_x(j), q_y(k), q_z(l));
            if(std::abs(q.norm()) > 1E-3) {
              q.normalize();
              Quaterniond quat(q(0), q(1), q(2), q(3));
              test_orientation_.push_back(AngleAxisd(quat));
            }
          }
        }
      }
    }*/
  }

  void setupRPYTestCases() {
    // Singularity issue associated with the second angle = pi/2
    // in Euler Body-fixed z-y'-x'' rotation sequence
    // Singularity issue associated with the second angle = -pi/2
    // in Euler Body-fixed z-y'-x'' rotation sequence
    // Singularity issue associated with the second angle close to pi/2
    // in Euler Body-fixed z-y'-x'' rotation sequence
    // Singularity issue associated with the second angle close to -pi/2
    // in Euler Body-fixed z-y'-x'' rotation sequence

    // pitch = pi/2
    rpy_test_cases_.push_back(Vector3d(M_PI/4, M_PI/2, M_PI/3));

    // pitch = -pi/2
    rpy_test_cases_.push_back(Vector3d(M_PI/4, -M_PI/2, M_PI/3));

    // pitch = (0.5-eps)*pi
    rpy_test_cases_.push_back(Vector3d(M_PI/4, (0.5 - numeric_limits<double>::epsilon())*M_PI, M_PI/3));

    // pitch = (0.5-1.5*eps)*pi
    rpy_test_cases_.push_back(Vector3d(M_PI/4, (0.5 - 1.5*numeric_limits<double>::epsilon())*M_PI, M_PI/3));

    // pitch = (0.5-2*eps)*pi
    rpy_test_cases_.push_back(Vector3d(M_PI/4, (0.5 - 2*numeric_limits<double>::epsilon())*M_PI, M_PI/3));

    // pitch = (0.5 - 1E-10)*pi
    rpy_test_cases_.push_back(Vector3d(M_PI*0.8, (0.5 - 1E-10)*M_PI, 0.9*M_PI));

    // pitch = (-0.5+eps)*pi
    rpy_test_cases_.push_back(Vector3d(M_PI*-0.9, (-0.5 + numeric_limits<double>::epsilon())*M_PI, M_PI*0.3));

    // pitch = (-0.5+1.5*eps)*pi
    rpy_test_cases_.push_back(Vector3d(M_PI*-0.6, (-0.5 + 1.5*numeric_limits<double>::epsilon())*M_PI, M_PI*0.3));

    // pitch = (-0.5+2*eps)*pi
    rpy_test_cases_.push_back(Vector3d(M_PI*-0.5, (-0.5 + 2*numeric_limits<double>::epsilon())*M_PI, M_PI*0.4));

    // pitch = (-0.5 + 1E-10)*pi
    rpy_test_cases_.push_back(Vector3d(M_PI*0.9, (-0.5 + 1E-10)*M_PI, 0.8*M_PI));

    // non-singular cases
    auto roll = Eigen::VectorXd::LinSpaced(Eigen::Sequential, 10, -M_PI, M_PI);
    auto pitch = Eigen::VectorXd::LinSpaced(Eigen::Sequential, 10, -0.49*M_PI, 0.49*M_PI);
    auto yaw = Eigen::VectorXd::LinSpaced(Eigen::Sequential, 10, -M_PI, M_PI);
    for(int i = 0; i < roll.size(); ++i) {
      for (int j = 0; j < pitch.size(); ++j) {
        for (int k = 0; k < yaw.size(); ++k) {
          rpy_test_cases_.push_back(Vector3d(roll(i), pitch(j), yaw(j)));
        }
      }
    }
  }

  void addAngleAxisTestCase(double angle, const Vector3d &axis) {
    angle_axis_test_cases_.push_back(AngleAxisd(angle, axis));
  }
  void setupAngleAxisTestCases() {
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
    Vector3d axis(0.5*sqrt(2), 0.4*sqrt(2), 0.3*sqrt(2));
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
    addAngleAxisTestCase((1-numeric_limits<double>::epsilon())*M_PI, axis);

    // (-1+epsilon)*pi rotation around an arbitrary axis
    addAngleAxisTestCase((-1+numeric_limits<double>::epsilon())*M_PI, axis);

    // (1-2*epsilon)*pi rotation around an arbitrary axis
    addAngleAxisTestCase((1-2*numeric_limits<double>::epsilon())*M_PI, axis);

    // (-1+2*epsilon)*pi rotation around an arbitrary axis
    addAngleAxisTestCase((-1+2*numeric_limits<double>::epsilon())*M_PI, axis);

    // (1-1E-10)*pi rotation around an arbitrary axis
    addAngleAxisTestCase((1-1E-10)*M_PI, axis);

    // (-1+1E-10)*pi rotation around an arbitrary axis
    addAngleAxisTestCase((-1+1E-10)*M_PI, axis);

    // non-singularity cases
    auto a_x = Eigen::VectorXd::LinSpaced(Eigen::Sequential, 10, -1, 1);
    auto a_y = Eigen::VectorXd::LinSpaced(Eigen::Sequential, 10, -1, 1);
    auto a_z = Eigen::VectorXd::LinSpaced(Eigen::Sequential, 10, -1, 1);
    auto a_angle = Eigen::VectorXd::LinSpaced(Eigen::Sequential, 10, -0.95*M_PI, 0.95*M_PI);
    for (int i = 0; i < a_x.size(); ++i) {
      for (int j = 0; j < a_y.size(); ++j) {
        for (int k = 0; k < a_z.size(); ++k) {
          Vector3d axis_ijk(a_x(i), a_y(j), a_z(k));
          if(axis_ijk.norm() > 1E-3) {
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
    auto qw = Eigen::VectorXd::LinSpaced(Eigen::Sequential, 10, -1, 1);
    auto qx = Eigen::VectorXd::LinSpaced(Eigen::Sequential, 10, -1, 1);
    auto qy = Eigen::VectorXd::LinSpaced(Eigen::Sequential, 10, -1, 1);
    auto qz = Eigen::VectorXd::LinSpaced(Eigen::Sequential, 10, -1, 1);
    for (int i = 0; i < qw.size(); ++i) {
      for (int j = 0; j < qx.size(); ++j) {
        for (int k = 0; k < qy.size(); ++k) {
          for (int l = 0; l < qz.size(); ++l) {
            Vector4d q(qw(i), qx(j), qy(k), qz(l));
            if(q.norm() > 1E-3) {
              q.normalize();
              quaternion_test_cases_.push_back(Quaterniond(q(0), q(1), q(2), q(3)));
            }
          }
        }
      }
    }
  }

  void setupRotationMatrixTestCases() {
    for(auto const &rpyi:rpy_test_cases_) {
      rotation_matrix_test_cases_.push_back(rpy2rotmat(rpyi));
    }
    for(auto const &ai:angle_axis_test_cases_) {
      rotation_matrix_test_cases_.push_back(ai.toRotationMatrix());
    }
    for(auto const &qi:quaternion_test_cases_) {
      rotation_matrix_test_cases_.push_back(qi.toRotationMatrix());
    }
  }
  std::vector<Vector3d> rpy_test_cases_;
  std::vector<AngleAxisd> angle_axis_test_cases_;
  std::vector<Quaterniond> quaternion_test_cases_;
  std::vector<Matrix3d> rotation_matrix_test_cases_;
  std::vector<AngleAxisd> test_orientation_;  // test cases
};

TEST_F(RotationConversionTest, AxisQuat) {
  for (const auto& ai_eigen : angle_axis_test_cases_) {
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
  for (const auto& ai_eigen : angle_axis_test_cases_) {
    // Manually computes the rotation matrix from axis-angle representation,
    // using Rodriguez's rotation formula
    // https://en.wikipedia.org/wiki/Rodrigues%27_rotation_formula
    auto ai = eigenAxisToAxis(ai_eigen);
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
    EXPECT_TRUE(compareAngleAxis(ai_eigen, a_eigen_expected));
  }
}

TEST_F(RotationConversionTest, AxisRPY) {
  for (const auto ai_eigen : angle_axis_test_cases_) {
    auto ai = eigenAxisToAxis(ai_eigen);
    auto rpy = axis2rpy(ai);
    // axis2rpy should be the inversion of rpy2axis
    auto a_expected = rpy2axis(rpy);
    AngleAxisd a_eigen_expected(a_expected(3), a_expected.head<3>());
    EXPECT_TRUE(compareAngleAxis(ai_eigen, a_eigen_expected));
    EXPECT_TRUE(check_rpy_range(rpy));
  }
}

TEST_F(RotationConversionTest, QuatAxis) {
  for (const auto& qi_eigen : quaternion_test_cases_) {
    // Compute the angle-axis representation using Eigen geometry module,
    // compare the result with quat2axis
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
  for (const auto& qi_eigen : quaternion_test_cases_) {
    // Compute the rotation matrix using Eigen geometry module, compare the
    // result with quat2rotmat
    Vector4d qi = eigenQuaterniontoQuat(qi_eigen);
    auto rotmat_expected = qi_eigen.toRotationMatrix();
    auto rotmat = quat2rotmat(qi);
    EXPECT_TRUE(CompareMatrices(rotmat_expected, rotmat, 1E-10,
                                MatrixCompareType::absolute));
    // quat2rotmat should be the inversion of rotmat2quat
    auto quat_expected = rotmat2quat(rotmat);
    EXPECT_TRUE(compareQuaternion(qi, quat_expected));
  }
}

TEST_F(RotationConversionTest, QuatRPY) {
  for (const auto& qi_eigen : quaternion_test_cases_) {
    Vector4d qi = eigenQuaterniontoQuat(qi_eigen);
    auto rpy = quat2rpy(qi);
    // quat2rpy should be the inversion of rpy2quat
    auto quat_expected = rpy2quat(rpy);
    EXPECT_TRUE(compareQuaternion(qi, quat_expected));
    EXPECT_TRUE(check_rpy_range(rpy));
  }
}

TEST_F(RotationConversionTest, QuatEigenQuaternion) {
  for (const auto& qi_eigen : quaternion_test_cases_) {
    Vector4d qi = eigenQuaterniontoQuat(qi_eigen);
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
    auto quat = rotmat2quat(Ri);
    auto quat_expected_eigen = Quaterniond(Ri);
    auto quat_expectd = eigenQuaterniontoQuat(quat_expected_eigen);
    EXPECT_TRUE(compareQuaternion(quat, quat_expectd));
    // rotmat2quat should be the inversion of quat2rotmat
    auto rotmat_expected = quat2rotmat(quat);
    EXPECT_TRUE(Ri.isApprox(rotmat_expected));
  }
}

TEST_F(RotationConversionTest, RotmatAxis) {
  // Compute angle-axis using Eigen geometry module, compare the result with
  // rotmat2axis
  for (const auto& Ri : rotation_matrix_test_cases_) {
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
  for (const auto& Ri : rotation_matrix_test_cases_) {
    auto rpy = rotmat2rpy(Ri);
    // rotmat2rpy should be the inversion of rpy2rotmat
    auto rotmat_expected = rpy2rotmat(rpy);
    EXPECT_TRUE(Ri.isApprox(rotmat_expected, 1E-10));
    EXPECT_TRUE(check_rpy_range(rpy));
  }
}
TEST_F(RotationConversionTest, RPYRotmat) {
  // Compute the rotation matrix by rotz(rpy(2))*roty(rpy(1))*rotx(rpy(0)),
  // then compare the result with rpy2rotmat
  for (const auto& rpyi : rpy_test_cases_) {
    auto rotation_expected = Eigen::AngleAxisd(rpyi(2), Vector3d::UnitZ()) *
        Eigen::AngleAxisd(rpyi(1), Vector3d::UnitY()) *
        Eigen::AngleAxisd(rpyi(0), Vector3d::UnitX());
    auto rotmat = rpy2rotmat(rpyi);
    EXPECT_TRUE(CompareMatrices(rotmat, rotation_expected.toRotationMatrix(),
                                1E-10, MatrixCompareType::absolute));
    // rpy2rotmat should be the inversion of rotmat2rpy
    Vector3d rpy_expected = rotmat2rpy(rotmat);
    EXPECT_TRUE(compareRollPitchYaw(rpyi, rpy_expected));
  }
}

TEST_F(RotationConversionTest, RPYAxis) {
  // Compute the angle-axis representation using Eigen's geometry module,
  // compare the result with rpy2axis
  for (const auto& rpyi : rpy_test_cases_) {
    auto rotation_expected = Eigen::AngleAxisd(rpyi(2), Vector3d::UnitZ()) *
        Eigen::AngleAxisd(rpyi(1), Vector3d::UnitY()) *
        Eigen::AngleAxisd(rpyi(0), Vector3d::UnitX());
    auto a_eigen_expected = Eigen::AngleAxis<double>(rotation_expected);
    auto a = rpy2axis(rpyi);
    AngleAxisd a_eigen = axisToEigenAngleAxis(a);
    EXPECT_TRUE(compareAngleAxis(a_eigen, a_eigen_expected));

    // rpy2axis should be the inversion of axis2rpy
    Vector3d rpy_expected = axis2rpy(a);
    if(!compareRollPitchYaw(rpyi, rpy_expected)) {
      auto Ri = rpy2rotmat(rpyi);
      auto rpyi_inverse = rotmat2rpy(Ri);
      if(!compareRollPitchYaw(rpyi, rpyi_inverse)) {
        EXPECT_TRUE(false);
      }
      auto R_expected = rpy2rotmat(rpy_expected);
      auto rpy_inverse = rotmat2rpy(R_expected);
      if(!compareRollPitchYaw(rpy_expected, rpy_inverse)) {
        EXPECT_TRUE(false);
      }
    }
    EXPECT_TRUE(compareRollPitchYaw(rpyi, rpy_expected));
  }
}

TEST_F(RotationConversionTest, RPYQuat) {
  // Compute the quaternion representation using Eigen's geometry model,
  // compare the result with rpy2quat
  for (const auto& rpyi : rpy_test_cases_) {
    Vector3d euler(rpyi(2), rpyi(1), rpyi(0));
    auto quat_eigen_expected = eulerToQuaternion(euler);
    auto quat_expected = eigenQuaterniontoQuat(quat_eigen_expected);
    auto quat = rpy2quat(rpyi);
    compareQuaternion(quat, quat_expected);
    // rpy2quat should be the inversion of quat2rpy
    auto rpy_expected = quat2rpy(quat);
    EXPECT_TRUE(compareRollPitchYaw(rpyi, rpy_expected));
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
