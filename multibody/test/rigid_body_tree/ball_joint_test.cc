#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/common/find_resource.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/multibody/joints/floating_base_types.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_tree.h"

namespace drake {
namespace systems {
namespace plants {
namespace test {
namespace {

// Loads a 2 link robot whose 2 links are connect by a ball joint, and the
// first link is welded to the world, tests the kinematics of the second link
// for an arbitrarily set q and v for the ball joint.
GTEST_TEST(RigidBodyTreeTest, BallJointTest) {
  RigidBodyTree<double> tree;
  const std::string filename = FindResourceOrThrow(
      "drake/multibody/test/rigid_body_tree/two_link_ball_joint_robot.urdf");
  parsers::urdf::AddModelInstanceFromUrdfFileToWorld(
      filename, multibody::joints::kFixed, &tree);

  // Checks dimensions.
  EXPECT_EQ(tree.get_num_positions(), 4);
  EXPECT_EQ(tree.get_num_velocities(), 3);

  Matrix3<double> rot(AngleAxis<double>(0.3, Vector3<double>::UnitX()) *
                      AngleAxis<double>(-3.2, Vector3<double>::UnitY()) *
                      AngleAxis<double>(0.88, Vector3<double>::UnitZ()));
  Quaternion<double> quat(rot);

  // Q is using Drake's quaternion which is w, x, y, z.
  VectorX<double> q = Vector4<double>(quat.w(), quat.x(), quat.y(), quat.z());
  // V is angular velocity in the body frame.
  VectorX<double> v = Vector3<double>(1, -2.33, 3.14);

  KinematicsCache<double> cache = tree.doKinematics(q, v);

  // Get the second link's (connected to the first link through a ball joint)
  // pose and velocity in the world frame.
  const RigidBody<double>& body = *tree.get_bodies()[2];
  Isometry3<double> X_WB = tree.CalcBodyPoseInWorldFrame(cache, body);
  Vector6<double> V_WB = tree.CalcBodySpatialVelocityInWorldFrame(cache, body);

  // Since the second link is co-located at the ball joint, its pose in the
  // world frame should have the same position as the tip of the first
  // link (0, 0, 0.6), and the orientation should match rot.
  const double kTol = 1e-12;
  EXPECT_TRUE(
      CompareMatrices(X_WB.linear(), rot, kTol, MatrixCompareType::absolute));
  EXPECT_TRUE(CompareMatrices(X_WB.translation(), Vector3<double>(0, 0, 0.6),
                              kTol, MatrixCompareType::absolute));

  // Its angular velocity in the world frame should be rot * v, and linear
  // velocity should be zero.
  EXPECT_TRUE(CompareMatrices(rot * v, V_WB.head<3>(), kTol,
                              MatrixCompareType::absolute));
  EXPECT_TRUE(CompareMatrices(Vector3<double>::Zero(), V_WB.tail<3>(), kTol,
                              MatrixCompareType::absolute));

  // Since V_WB = J * v, the angular part of J = rot, and linear part is zero.
  Matrix6X<double> J = tree.CalcFrameSpatialVelocityJacobianInWorldFrame(
      cache, body, Isometry3<double>::Identity());
  EXPECT_EQ(J.cols(), 3);
  EXPECT_TRUE(CompareMatrices(J.block<3, 3>(0, 0), rot, kTol,
                              MatrixCompareType::absolute));
  EXPECT_TRUE(CompareMatrices(J.block<3, 3>(3, 0), Matrix3<double>::Zero(),
                              kTol, MatrixCompareType::absolute));

  // Since for this robot, A_WB = rot * vd = rot * vd + JdotV. JdotV = 0.
  Vector6<double> JdotV =
      tree.CalcFrameSpatialVelocityJacobianDotTimesVInWorldFrame(
          cache, body, Isometry3<double>::Identity());
  EXPECT_TRUE(CompareMatrices(JdotV, Vector6<double>::Zero(), kTol,
                              MatrixCompareType::absolute));
}

}  // namespace
}  // namespace test
}  // namespace plants
}  // namespace systems
}  // namespace drake
