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
using Drake::RigidBodyMagnetometer;
using Drake::RigidBodySystem;
using drake::util::MatrixCompareType;

namespace drake {
namespace systems {
namespace plants {
namespace {

Vector3d getMagnetometerOutput(shared_ptr<RigidBodySystem> const& sys,
                               Vector3d const& rpy) {
  VectorXd x0 = VectorXd::Zero(sys->getNumStates());
  auto const& tree = sys->getRigidBodyTree();
  x0.head(tree->number_of_positions()) = tree->getZeroConfiguration();
  x0.segment(3, 4) = ::drake::math::rpy2quat(rpy);
  auto const& system_output = sys->output(0, x0, Vector4d::Zero());
  return system_output.tail<3>();
}

GTEST_TEST(testMagnetometer, AllTests) {
  DrakeJoint::FloatingBaseType floating_base_type = DrakeJoint::QUATERNION;
  auto rigid_body_sys = make_shared<RigidBodySystem>();
  rigid_body_sys->addRobotFromFile(
      getDrakePath() + "/examples/Quadrotor/quadrotor.urdf",
      floating_base_type);

  auto const& tree = rigid_body_sys->getRigidBodyTree();
  auto sensor_frame = tree->findFrame("body");

  auto magnetometer = make_shared<RigidBodyMagnetometer>(
      *rigid_body_sys, "magnetometer", sensor_frame, 0.0);
  rigid_body_sys->addSensor(magnetometer);

  const double tol = 1e-6;

  EXPECT_TRUE(
      CompareMatrices(getMagnetometerOutput(rigid_body_sys, Vector3d::Zero()),
                      Vector3d(1, 0, 0), tol, MatrixCompareType::absolute));

  EXPECT_TRUE(CompareMatrices(
      getMagnetometerOutput(rigid_body_sys, Vector3d(0, 0, M_PI / 2)),
      Vector3d(0, -1, 0), tol, MatrixCompareType::absolute));

  magnetometer->setDeclination(M_PI / 4);

  EXPECT_TRUE(
      CompareMatrices(getMagnetometerOutput(rigid_body_sys, Vector3d::Zero()),
                      Vector3d(std::sqrt(2) / 2, std::sqrt(2) / 2, 0), tol,
                      MatrixCompareType::absolute));

  EXPECT_TRUE(CompareMatrices(
      getMagnetometerOutput(rigid_body_sys, Vector3d(0, 0, M_PI / 2)),
      Vector3d(std::sqrt(2) / 2, -std::sqrt(2) / 2, 0), tol,
      MatrixCompareType::absolute));
}

}  // namespace
}  // namespace plants
}  // namespace systems
}  // namespace drake
