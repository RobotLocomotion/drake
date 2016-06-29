#include "gtest/gtest.h"

#include "drake/math/roll_pitch_yaw.h"
#include "drake/Path.h"
#include "drake/systems/plants/RigidBodySystem.h"
#include "drake/util/eigen_matrix_compare.h"
#include "drake/util/testUtil.h"

using Eigen::Vector3d;
using Eigen::Vector4d;
using Eigen::VectorXd;
using std::shared_ptr;
using std::make_shared;
using Drake::getDrakePath;
using Drake::RigidBodySystem;
using drake::util::MatrixCompareType;
using Drake::RigidBodyAccelerometer;

namespace drake {
namespace systems {
namespace plants {
namespace {

Vector3d getAccelerometerOutput(shared_ptr<RigidBodySystem> const& sys,
                                Vector3d const& rpy, Vector4d const& u) {
  VectorXd x0 = VectorXd::Zero(sys->getNumStates());
  auto const& tree = sys->getRigidBodyTree();
  x0.head(tree->number_of_positions()) = tree->getZeroConfiguration();
  x0.segment(3, 4) = ::drake::math::rpy2quat(rpy);
  auto const& system_output = sys->output(0, x0, u);
  return system_output.tail<3>();
}

GTEST_TEST(testAccelerometer, AllTests) {
  DrakeJoint::FloatingBaseType floating_base_type = DrakeJoint::QUATERNION;
  auto rigid_body_sys = make_shared<RigidBodySystem>();
  rigid_body_sys->addRobotFromFile(
      getDrakePath() + "/examples/Quadrotor/quadrotor.urdf",
      floating_base_type);
  auto const& tree = rigid_body_sys->getRigidBodyTree();
  auto sensor_frame = tree->findFrame("body");
  auto accelerometer = make_shared<RigidBodyAccelerometer>(
      *rigid_body_sys, "accelerometer", sensor_frame);
  rigid_body_sys->addSensor(accelerometer);
  const double g = 9.81;
  const double tol = 1e-6;
  const Vector4d hoverThrust = 1.226250 * Vector4d::Ones();

  EXPECT_TRUE(
      CompareMatrices(getAccelerometerOutput(rigid_body_sys, Vector3d::Zero(),
                                             Vector4d::Zero()),
                      Vector3d(0, 0, -g), tol, MatrixCompareType::absolute));

  EXPECT_TRUE(CompareMatrices(
      getAccelerometerOutput(rigid_body_sys, Vector3d(M_PI, 0, 0),
                             Vector4d::Zero()),  // inverted
      Vector3d(0, 0, g),
      tol, MatrixCompareType::absolute));

  EXPECT_TRUE(CompareMatrices(
      getAccelerometerOutput(rigid_body_sys, Vector3d::Zero(), hoverThrust),
      Vector3d(0, 0, 0), tol, MatrixCompareType::absolute));

  EXPECT_TRUE(CompareMatrices(
      getAccelerometerOutput(rigid_body_sys, Vector3d(M_PI, 0, 0),
                             hoverThrust),  // inverted with thrust
      Vector3d(0, 0, 2 * g),
      tol, MatrixCompareType::absolute));

  EXPECT_TRUE(CompareMatrices(
      getAccelerometerOutput(rigid_body_sys, Vector3d(M_PI / 2, 0, 0),
                             Vector4d::Zero()),
      Vector3d(0, -g, 0), tol, MatrixCompareType::absolute));

  EXPECT_TRUE(CompareMatrices(
      getAccelerometerOutput(rigid_body_sys, Vector3d(-M_PI / 2, 0, 0),
                             Vector4d::Zero()),
      Vector3d(0, g, 0), tol, MatrixCompareType::absolute));

  EXPECT_TRUE(CompareMatrices(
      getAccelerometerOutput(rigid_body_sys, Vector3d(0, M_PI / 2, 0),
                             Vector4d::Zero()),
      Vector3d(g, 0, 0), tol, MatrixCompareType::absolute));

  EXPECT_TRUE(CompareMatrices(
      getAccelerometerOutput(rigid_body_sys, Vector3d(0, -M_PI / 2, 0),
                             Vector4d::Zero()),
      Vector3d(-g, 0, 0), tol, MatrixCompareType::absolute));

  // real accelerometers can't measure gravity during freefall
  // but will measure gravity while hovering
  accelerometer->setGravityCompensation(true);
  EXPECT_TRUE(
      CompareMatrices(getAccelerometerOutput(rigid_body_sys, Vector3d::Zero(),
                                             Vector4d::Zero()),
                      Vector3d(0, 0, 0), tol, MatrixCompareType::absolute));

  EXPECT_TRUE(CompareMatrices(
      getAccelerometerOutput(rigid_body_sys, Vector3d(M_PI, 0, 0),
                             Vector4d::Zero()),
      Vector3d(0, 0, 0), tol, MatrixCompareType::absolute));

  EXPECT_TRUE(CompareMatrices(
      getAccelerometerOutput(rigid_body_sys, Vector3d::Zero(), hoverThrust),
      Vector3d(0, 0, g), tol, MatrixCompareType::absolute));
}

}  // namespace
}  // namespace plants
}  // namespace systems
}  // namespace drake
