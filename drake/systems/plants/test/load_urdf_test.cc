#include <iostream>

#include <gtest/gtest.h>

// #include "drake/Path.h"
#include "drake/systems/plants/RigidBodySystem.h"
#include "drake/systems/plants/RigidBodyFrame.h"
#include "drake/util/eigen_matrix_compare.h"
#include "drake/util/testUtil.h"

using std::make_shared;
using Drake::RigidBodySystem;
using Eigen::VectorXd;
using drake::util::MatrixCompareType;

namespace drake {
namespace systems {
namespace plants {
namespace test {
namespace {

std::string model_file_1, model_file_2;
std::shared_ptr<RigidBodyFrame> car_pose_in_world;

TEST(LoadURDFTest, TestNoOffset) {
  // Loads the URDF model with zero offset between the model's
  // root frame and the world frame.
  RigidBodySystem rbs;
  rbs.addRobotFromFile(Drake::getDrakePath()
    + "/systems/plants/test/cylindrical_1dof_robot.urdf", DrakeJoint::QUATERNION);

  // Verifies that RigidBodyTree cannot find a link that
  // should not exist.
  auto nonexistent_body = rbs.getRigidBodyTree()->findLink("not_a_link");
  EXPECT_TRUE(nonexistent_body == nullptr);

  // Verifies that the world link within the rigid body tree
  // can be found and obtained.
  auto world_body = rbs.getRigidBodyTree()->findLink("world");
  EXPECT_TRUE(world_body != nullptr);

  // Verifies that the world link does not have a parent.
  EXPECT_FALSE(world_body->hasParent());

  // Gets the link whose parent joint is called "base".
  auto link1_body = rbs.getRigidBodyTree()->findJoint("base");
  EXPECT_TRUE(link1_body != nullptr);
  EXPECT_EQ(link1_body->linkname.compare("link1"), 0);

  // Verifies that the transformation from link1's frame to the world's frame
  // is correct. Since the model was not offset from the world, we expect
  // the transformation to be Eigen::Isometry3d::Identity().
  EXPECT_EQ(link1_body->getJoint().getTransformToParentBody().matrix(),
    Eigen::Isometry3d::Identity().matrix());

  // Gets the link whose parent joint is called "joint1".
  auto link2_body = rbs.getRigidBodyTree()->findJoint("joint1");
  EXPECT_TRUE(link2_body != nullptr);
  EXPECT_EQ(link2_body->linkname.compare("link2"), 0);

  // Verifies that the transformation from link2's frame to link1's frame is
  // correct. From the URDF, the transformation is expected to be
  // x = 0, y = 0, z = 0.6.
  Eigen::Isometry3d T_link2_to_link1;
  {
    Eigen::Vector3d rpy = Eigen::Vector3d::Zero(), xyz = Eigen::Vector3d::Zero();
    xyz(2) = 0.6;
    T_link2_to_link1.matrix() << rpy2rotmat(rpy), xyz, 0, 0, 0, 1;
  }

  EXPECT_EQ(link2_body->getJoint().getTransformToParentBody().matrix(),
    T_link2_to_link1.matrix());
}

TEST(LoadURDFTest, TestVerticalOffset) {
  // Welds the robot to the world with a Z-offset of 1.0 m, X-offset of 1.0m,
  // and Y-offset of 1.0m.
  Eigen::Isometry3d T_model_to_world;
  {
    Eigen::Vector3d rpy = Eigen::Vector3d::Zero(), xyz = Eigen::Vector3d::Ones();
    T_model_to_world.matrix() << rpy2rotmat(rpy), xyz, 0, 0, 0, 1;
  }

  auto weld_to_frame = std::allocate_shared<RigidBodyFrame>(
      Eigen::aligned_allocator<RigidBodyFrame>(), "world",
      nullptr,  // not used since the robot is attached to the world
      T_model_to_world);

  RigidBodySystem rbs;
  rbs.addRobotFromFile(Drake::getDrakePath()
    + "/systems/plants/test/cylindrical_1dof_robot.urdf", DrakeJoint::QUATERNION,
    weld_to_frame);

  // Gets the link whose parent joint is called "base".
  auto link1_body = rbs.getRigidBodyTree()->findJoint("base");
  EXPECT_TRUE(link1_body != nullptr);
  EXPECT_EQ(link1_body->linkname.compare("link1"), 0);

  // Verifies that the transformation from link1's frame to the world's frame
  // is correct. Since the model was offset from the world frame, we expect the
  // transformation of link1_body's frame to the world frame to be equal to
  // T_model_to_world.
  EXPECT_EQ(link1_body->getJoint().getTransformToParentBody().matrix(),
    T_model_to_world.matrix());

  // Gets the link whose parent joint is called "joint1".
  auto link2_body = rbs.getRigidBodyTree()->findJoint("joint1");
  EXPECT_TRUE(link2_body != nullptr);
  EXPECT_EQ(link2_body->linkname.compare("link2"), 0);

  // Verifies that the transformation from link2's frame to link1's frame is
  // correct. From the URDF, the transformation is expected to be
  // x = 0, y = 0, z = 0.6.
  Eigen::Isometry3d T_link2_to_link1;
  {
    Eigen::Vector3d rpy = Eigen::Vector3d::Zero(), xyz = Eigen::Vector3d::Zero();
    xyz(2) = 0.6;
    T_link2_to_link1.matrix() << rpy2rotmat(rpy), xyz, 0, 0, 0, 1;
  }

  EXPECT_EQ(link2_body->getJoint().getTransformToParentBody().matrix(),
    T_link2_to_link1.matrix());
}

TEST(LoadURDFTest, TestWeld) {
  // Loads a one-DOF URDF model with zero offset between the model's
  // root frame and the world frame.
  RigidBodySystem rbs;
  rbs.addRobotFromFile(Drake::getDrakePath()
    + "/systems/plants/test/cylindrical_1dof_robot.urdf", DrakeJoint::QUATERNION);

  // Loads a zero-DOF URDF robot model and weld it to the end of the
  // one DOF robot's link 2.
  Eigen::Isometry3d T_model2_to_link2;
  {
    Eigen::Vector3d rpy = Eigen::Vector3d::Zero();
    Eigen::Vector3d xyz = Eigen::Vector3d::Zero();
    xyz(2) = 0.06;
    T_model2_to_link2.matrix() << rpy2rotmat(rpy), xyz, 0, 0, 0, 1;
  }

  auto link2_body = rbs.getRigidBodyTree()->findLink("link2");
  EXPECT_TRUE(link2_body != nullptr);

  auto weld_to_frame = std::allocate_shared<RigidBodyFrame>(
      Eigen::aligned_allocator<RigidBodyFrame>(), "joint2",
      link2_body,
      T_model2_to_link2);
  rbs.addRobotFromFile(Drake::getDrakePath()
    + "/systems/plants/test/cylindrical_0dof_robot.urdf", DrakeJoint::FIXED,
    weld_to_frame);

  // Verifies that the newly added link exists and is in the correct location.
  auto link_body = rbs.getRigidBodyTree()->findLink("link");
  EXPECT_TRUE(link_body != nullptr);
  EXPECT_EQ(link_body->getJoint().getTransformToParentBody().matrix(),
    T_model2_to_link2.matrix());
}


}  // namespace
}  // namespace test
}  // namespace plants
}  // namespace systems
}  // namespace drake
