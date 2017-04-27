#include <gtest/gtest.h>

#include "drake/common/eigen_matrix_compare.h"
#include "drake/examples/QPInverseDynamicsForHumanoids/control_utils.h"

namespace drake {
namespace examples {
namespace qp_inverse_dynamics {

// Test rotational difference between the desired and measured orientation.
// They are both generated by rotating around the same vector, and they differ
// in the angle of rotation.
GTEST_TEST(testQPInverseDynamicsController, testPoseSetpoint) {
  // Desired values are specified with suffix "_d"
  Isometry3<double> pose_d = Isometry3<double>::Identity();
  double ang_d = 0.3;
  Vector3<double> vec_d(-0.3, 0.6, 0.9);
  vec_d.normalize();
  // Desired orientation
  pose_d.linear() = Matrix3<double>(AngleAxis<double>(ang_d, vec_d));

  // Set Kp to 1, and everything else to zeros, so the computed acceleration
  // is the rotation difference.
  CartesianSetpoint<double> setpoint(
      pose_d, Vector6<double>::Zero(), Vector6<double>::Zero(),
      Vector6<double>::Constant(1), Vector6<double>::Zero());
  Isometry3<double> pose = pose_d;
  Vector6<double> acc, expected;
  expected.setZero();

  for (double ang = ang_d; ang < ang_d + 2 * M_PI + 0.1; ang += 0.1) {
    pose.linear() = Matrix3<double>(AngleAxis<double>(ang, vec_d));
    acc = setpoint.ComputeTargetAcceleration(pose, Vector6<double>::Zero());

    double err = ang_d - ang;
    if (err > M_PI)
      err -= 2 * M_PI;
    else if (err < -M_PI)
      err += 2 * M_PI;

    expected.head<3>() = err * vec_d;
    EXPECT_TRUE(
        CompareMatrices(acc, expected, 1e-8, MatrixCompareType::absolute));
  }
}

}  // namespace qp_inverse_dynamics
}  // namespace examples
}  // namespace drake
