#include <cstdlib>
#include <iostream>

#include <gtest/gtest.h>

#include "drake/common/drake_path.h"
#include "drake/common/eigen_types.h"
#include "drake/common/eigen_matrix_compare.h"

#include "drake/math/roll_pitch_yaw.h"
#include "drake/multibody/rigid_body_plant/kinematics_results.h"
#include "drake/multibody/joints/floating_base_types.h"

namespace drake {
namespace systems {
namespace {

GTEST_TEST(KinematicsResultsTest, Test) {
  std::string file_name = drake::GetDrakePath() +
                          "/multibody/test/rigid_body_tree/two_dof_robot.urdf";
  RigidBodyTree<double> robot(file_name, multibody::joints::kRollPitchYaw);

  KinematicsResults<double> kin_res(&robot);

  // Set state.
  std::default_random_engine generator;
  VectorX<double> q = robot.getRandomConfiguration(generator);
  VectorX<double> v = VectorX<double>::Random(robot.get_num_velocities());

  // Update kinematics result.
  kin_res.Update(q, v);

  // Test pose.
  const RigidBody<double>& body = *robot.FindBody("link1");
  Isometry3<double> pose = kin_res.get_pose_in_world(body);
  Matrix3<double> rotmat = math::rpy2rotmat(q.segment<3>(3));

  // Position and orientation get from kinematics result = quantities directly
  // computed from q and v.
  EXPECT_TRUE(drake::CompareMatrices(q.segment<3>(0), pose.translation(), 1e-14,
                                     drake::MatrixCompareType::absolute));
  EXPECT_TRUE(drake::CompareMatrices(rotmat, pose.linear(), 1e-14,
                                     drake::MatrixCompareType::absolute));

  // Test vel.
  Vector6<double> v_in_world_aligned_body =
      kin_res.get_twist_in_world_aligned_body_frame(body);
  Vector6<double> v_in_world = kin_res.get_twist_in_world_frame(body);

  Vector3<double> rpy = q.segment<3>(3);
  Vector3<double> rpydot = v.segment<3>(3);
  Matrix3<double> phi = Matrix3<double>::Zero();
  angularvel2rpydotMatrix(rpy, phi, static_cast<Matrix3<double>*>(nullptr),
                          static_cast<Matrix3<double>*>(nullptr));
  Vector3<double> omega = phi.inverse() * rpydot;

  // v_in_world_aligned_body.angular = rpydot
  EXPECT_TRUE(
      drake::CompareMatrices(omega, v_in_world_aligned_body.segment<3>(0),
                             1e-14, drake::MatrixCompareType::absolute));
  // v_in_world_aligned_body.linear = xyzdot
  EXPECT_TRUE(drake::CompareMatrices(
      v.segment<3>(0), v_in_world_aligned_body.segment<3>(3), 1e-14,
      drake::MatrixCompareType::absolute));

  // v_in_world_aligned_body transformed as twist_in_world = twist_in_world from
  // kinematics result.
  Isometry3<double> world_aligned_to_world(Isometry3<double>::Identity());
  world_aligned_to_world.translation() = pose.translation();
  Vector6<double> twist_in_world =
      transformSpatialMotion(world_aligned_to_world, v_in_world_aligned_body);

  EXPECT_TRUE(drake::CompareMatrices(twist_in_world, v_in_world, 1e-14,
                                     drake::MatrixCompareType::absolute));

  // Joint position and velocity should match q and v.
  EXPECT_EQ(kin_res.get_joint_position(*robot.FindBody("link2"))[0], q[6]);
  EXPECT_EQ(kin_res.get_joint_position(*robot.FindBody("link3"))[0], q[7]);

  EXPECT_EQ(kin_res.get_joint_velocity(*robot.FindBody("link2"))[0], v[6]);
  EXPECT_EQ(kin_res.get_joint_velocity(*robot.FindBody("link3"))[0], v[7]);
}

}  // namespace
}  // namespace systems
}  // namespace drake
