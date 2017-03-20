#include "drake/multibody/rigid_body_plant/kinematics_results.h"

#include <cstdlib>
#include <iostream>
#include <memory>

#include <gtest/gtest.h>

#include "drake/common/drake_path.h"
#include "drake/common/eigen_matrix_compare.h"
#include "drake/common/eigen_types.h"
#include "drake/math/roll_pitch_yaw.h"
#include "drake/multibody/joints/floating_base_types.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/util/drakeGeometryUtil.h"

namespace drake {
namespace systems {
namespace {

class KinematicsResultsTest : public ::testing::Test {
 protected:
  void SetUp() override {
    std::string file_name =
        GetDrakePath() + "/multibody/test/rigid_body_tree/two_dof_robot.urdf";
    robot_ = std::make_unique<RigidBodyTree<double>>();
    parsers::urdf::AddModelInstanceFromUrdfFileToWorld(
        file_name, multibody::joints::kRollPitchYaw, robot_.get());
    kinematics_result_ =
        std::make_unique<KinematicsResults<double>>(robot_.get());

    q_ = VectorX<double>(robot_->get_num_positions());
    v_ = VectorX<double>(robot_->get_num_velocities());

    for (int i = 0; i < q_.size(); ++i) q_[i] = 0.2 * i;
    for (int i = 0; i < v_.size(); ++i) v_[i] = -0.2 * i;

    kinematics_result_->Update(q_, v_);
  }

  std::unique_ptr<RigidBodyTree<double>> robot_;
  std::unique_ptr<KinematicsResults<double>> kinematics_result_;
  VectorX<double> q_;
  VectorX<double> v_;
};

TEST_F(KinematicsResultsTest, PoseTest) {
  const RigidBody<double>& body = *robot_->FindBody("link1");
  Isometry3<double> pose = kinematics_result_->get_pose_in_world(body);
  Matrix3<double> rotmat = math::rpy2rotmat(q_.segment<3>(3));

  // Position and orientation get from kinematics result = quantities directly
  // computed from q and v.
  EXPECT_TRUE(drake::CompareMatrices(q_.segment<3>(0), pose.translation(),
                                     1e-14,
                                     drake::MatrixCompareType::absolute));
  EXPECT_TRUE(drake::CompareMatrices(rotmat, pose.linear(), 1e-14,
                                     drake::MatrixCompareType::absolute));
}

TEST_F(KinematicsResultsTest, TwistTest) {
  const RigidBody<double>& body = *robot_->FindBody("link1");
  Vector3<double> rpy = q_.segment<3>(3);
  Vector3<double> rpydot = v_.segment<3>(3);
  Vector3<double> xyzdot = v_.segment<3>(0);

  Isometry3<double> pose = kinematics_result_->get_pose_in_world(body);

  Vector6<double> v_in_world_aligned_body =
      kinematics_result_->get_twist_in_world_aligned_body_frame(body);
  Vector6<double> v_in_world =
      kinematics_result_->get_twist_in_world_frame(body);

  Matrix3<double> phi = Matrix3<double>::Zero();
  typename math::Gradient<decltype(phi), Eigen::Dynamic>::type* dphi = nullptr;
  typename math::Gradient<decltype(phi), Eigen::Dynamic, 2>::type* ddphi =
      nullptr;
  angularvel2rpydotMatrix(rpy, phi, dphi, ddphi);
  Vector3<double> omega = phi.inverse() * rpydot;

  // v_in_world_aligned_body.angular = rpydot
  EXPECT_TRUE(
      drake::CompareMatrices(omega, v_in_world_aligned_body.segment<3>(0),
                             1e-14, drake::MatrixCompareType::absolute));
  // v_in_world_aligned_body.linear = xyzdot
  EXPECT_TRUE(
      drake::CompareMatrices(xyzdot, v_in_world_aligned_body.segment<3>(3),
                             1e-14, drake::MatrixCompareType::absolute));

  // v_in_world_aligned_body transformed as twist_in_world = twist_in_world from
  // kinematics result.
  Isometry3<double> world_aligned_to_world(Isometry3<double>::Identity());
  world_aligned_to_world.translation() = pose.translation();
  Vector6<double> twist_in_world =
      transformSpatialMotion(world_aligned_to_world, v_in_world_aligned_body);

  EXPECT_TRUE(drake::CompareMatrices(twist_in_world, v_in_world, 1e-14,
                                     drake::MatrixCompareType::absolute));
}

TEST_F(KinematicsResultsTest, JointTest) {
  EXPECT_EQ(
      kinematics_result_->get_joint_position(*robot_->FindBody("link2"))[0],
      q_[6]);
  EXPECT_EQ(
      kinematics_result_->get_joint_position(*robot_->FindBody("link3"))[0],
      q_[7]);

  EXPECT_EQ(
      kinematics_result_->get_joint_velocity(*robot_->FindBody("link2"))[0],
      v_[6]);
  EXPECT_EQ(
      kinematics_result_->get_joint_velocity(*robot_->FindBody("link3"))[0],
      v_[7]);
}

}  // namespace
}  // namespace systems
}  // namespace drake
