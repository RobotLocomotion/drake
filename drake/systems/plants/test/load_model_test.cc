#include <iostream>

#include <gtest/gtest.h>

#include "drake/Path.h"
#include "drake/systems/plants/RigidBodyFrame.h"
#include "drake/systems/plants/RigidBodySystem.h"

using Drake::RigidBodySystem;

namespace drake {
namespace systems {
namespace plants {
namespace test {
namespace {

/**
 * Defines a class that extends Google Test's ::testing::TestWithParam
 * class, which enables parameterized Google Tests. In this case, the parameter
 * is a const char* and it is the file extension of the model files to use in
 * the LoadModelTest unit tests. Specifically, the possible values are "urdf"
 * and "sdf".
 */
class LoadModelTest : public ::testing::TestWithParam<const char*> {
 protected:
  RigidBodySystem rbs;
};

TEST_P(LoadModelTest, TestNoOffset) {
  // Loads the SDF model with zero offset between the model's root frame and
  // the world frame.
  rbs.addRobotFromFile(Drake::getDrakePath() +
                           "/systems/plants/test/cylindrical_1dof_robot." +
                           GetParam(),
                       DrakeJoint::QUATERNION);

  // Verifies that RigidBodyTree cannot find a link thatn does not exist.
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
  EXPECT_EQ(link1_body->name_, "link1");

  // Verifies that the transformation from link1's frame to the world's frame
  // is correct. Since the model was not offset from the world, we expect
  // the transformation to be Eigen::Isometry3d::Identity().
  EXPECT_TRUE(link1_body->getJoint().getTransformToParentBody().matrix() ==
              Eigen::Isometry3d::Identity().matrix());

  // Gets the link whose parent joint is called "joint1".
  auto link2_body = rbs.getRigidBodyTree()->findJoint("joint1");
  EXPECT_TRUE(link2_body != nullptr);
  EXPECT_EQ(link2_body->name_, "link2");

  // Verifies that the transformation from link2's frame to link1's frame is
  // correct. From the SDF, the transformation is expected to be
  // x = 0, y = 0, z = 0.6.
  Eigen::Isometry3d T_link2_to_link1;
  {
    Eigen::Vector3d rpy = Eigen::Vector3d::Zero(),
                    xyz = Eigen::Vector3d::Zero();
    xyz(2) = 0.6;
    T_link2_to_link1.matrix() << rpy2rotmat(rpy), xyz, 0, 0, 0, 1;
  }

  EXPECT_TRUE(link2_body->getJoint().getTransformToParentBody().matrix() ==
              T_link2_to_link1.matrix());
}

TEST_P(LoadModelTest, TestVerticalOffset) {
  // Welds the robot to the world with a Z-offset of 1.0 m, X-offset of 1.0m,
  // and Y-offset of 1.0m.
  Eigen::Isometry3d T_model_to_world;
  {
    Eigen::Vector3d rpy = Eigen::Vector3d::Zero(),
                    xyz = Eigen::Vector3d::Ones();
    T_model_to_world.matrix() << rpy2rotmat(rpy), xyz, 0, 0, 0, 1;
  }

  auto weld_to_frame = std::allocate_shared<RigidBodyFrame>(
      Eigen::aligned_allocator<RigidBodyFrame>(), "world",
      nullptr,  // not used since the robot is attached to the world
      T_model_to_world);

  rbs.addRobotFromFile(Drake::getDrakePath() +
                           "/systems/plants/test/cylindrical_1dof_robot." +
                           GetParam(),
                       DrakeJoint::QUATERNION, weld_to_frame);

  // Gets the link whose parent joint is called "base".
  auto link1_body = rbs.getRigidBodyTree()->findJoint("base");
  EXPECT_TRUE(link1_body != nullptr);
  EXPECT_EQ(link1_body->name_, "link1");

  // Verifies that the transformation from link1's frame to the world's frame
  // is correct. Since the model was offset from the world frame, we expect the
  // transformation of link1_body's frame to the world frame to be equal to
  // T_model_to_world.
  EXPECT_TRUE(link1_body->getJoint().getTransformToParentBody().matrix() ==
              T_model_to_world.matrix());

  // Gets the link whose parent joint is called "joint1".
  auto link2_body = rbs.getRigidBodyTree()->findJoint("joint1");
  EXPECT_TRUE(link2_body != nullptr);
  EXPECT_EQ(link2_body->name_, "link2");

  // Verifies that the transformation from link2's frame to link1's frame is
  // correct. From the SDF, the transformation is expected to be
  // x = 0, y = 0, z = 0.6.
  Eigen::Isometry3d T_link2_to_link1;
  {
    Eigen::Vector3d rpy = Eigen::Vector3d::Zero(),
                    xyz = Eigen::Vector3d::Zero();
    xyz(2) = 0.6;
    T_link2_to_link1.matrix() << rpy2rotmat(rpy), xyz, 0, 0, 0, 1;
  }

  EXPECT_TRUE(link2_body->getJoint().getTransformToParentBody().matrix() ==
              T_link2_to_link1.matrix());
}

TEST_P(LoadModelTest, TestWeld) {
  // Loads a one-DOF SDF model with zero offset between the model's
  // root frame and the world frame.
  rbs.addRobotFromFile(Drake::getDrakePath() +
                           "/systems/plants/test/cylindrical_1dof_robot." +
                           GetParam(),
                       DrakeJoint::QUATERNION);

  // Loads a zero-DOF SDF robot model and weld it to the end of the
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
      Eigen::aligned_allocator<RigidBodyFrame>(), "joint2", link2_body,
      T_model2_to_link2);
  rbs.addRobotFromFile(Drake::getDrakePath() +
                           "/systems/plants/test/cylindrical_0dof_robot." +
                           GetParam(),
                       DrakeJoint::FIXED, weld_to_frame);

  // Verifies that the newly added link exists and is in the correct location.
  auto link_body = rbs.getRigidBodyTree()->findLink("link");
  EXPECT_TRUE(link_body != nullptr);
  EXPECT_TRUE(link_body->getJoint().getTransformToParentBody().matrix() ==
              T_model2_to_link2.matrix());
}

INSTANTIATE_TEST_CASE_P(LOAD_SDF_AND_URF_TESTS, LoadModelTest,
                        ::testing::Values("urdf", "sdf"));

TEST(LoadSDFTest, TestInternalOffset) {
  // Loads a one-DOF SDF model with:
  //   1. A Z = 1 offset between the model's root and the model's world
  //   2. Zero offset between the model's world and Drake's world
  RigidBodySystem rbs;
  rbs.addRobotFromFile(
      Drake::getDrakePath() +
          "/systems/plants/test/cylindrical_1dof_robot_offset_z1.sdf",
      DrakeJoint::QUATERNION);

  // Verifies that the transform between the robot's root node
  // and the world is equal to Z = 1.
  Eigen::Isometry3d T_model_to_world;
  {
    Eigen::Vector3d rpy = Eigen::Vector3d::Zero();
    Eigen::Vector3d xyz = Eigen::Vector3d::Zero();
    xyz(2) = 1;
    T_model_to_world.matrix() << rpy2rotmat(rpy), xyz, 0, 0, 0, 1;
  }

  auto link1_body = rbs.getRigidBodyTree()->findLink("link1");
  EXPECT_TRUE(link1_body != nullptr);
  EXPECT_TRUE(link1_body->getJoint().getTransformToParentBody().matrix() ==
              T_model_to_world.matrix());
}

TEST(LoadSDFTest, TestDualOffset1) {
  // Loads a one-DOF SDF model with:
  //   1. A Z = 1 offset between the model's root and the model's world
  //   2. An X = 2 offset between the model's world and Drake's world
  Eigen::Isometry3d T_model_world_to_drake_world;
  {
    Eigen::Vector3d rpy = Eigen::Vector3d::Zero();
    Eigen::Vector3d xyz = Eigen::Vector3d::Zero();
    xyz(0) = 2;
    T_model_world_to_drake_world.matrix() << rpy2rotmat(rpy), xyz, 0, 0, 0, 1;
  }

  auto weld_to_frame = std::allocate_shared<RigidBodyFrame>(
      Eigen::aligned_allocator<RigidBodyFrame>(), "world",
      nullptr,  // not used since the robot is attached to the world
      T_model_world_to_drake_world);

  RigidBodySystem rbs;
  rbs.addRobotFromFile(
      Drake::getDrakePath() +
          "/systems/plants/test/cylindrical_1dof_robot_offset_z1.sdf",
      DrakeJoint::QUATERNION, weld_to_frame);

  // Verifies that the transform between the robot's root node
  // and the world is equal to X = 2, Z = 1.
  Eigen::Isometry3d T_model_to_world;
  {
    Eigen::Vector3d rpy = Eigen::Vector3d::Zero();
    Eigen::Vector3d xyz;
    xyz << 2, 0, 1;
    T_model_to_world.matrix() << rpy2rotmat(rpy), xyz, 0, 0, 0, 1;
  }

  auto link1_body = rbs.getRigidBodyTree()->findLink("link1");
  EXPECT_TRUE(link1_body != nullptr);
  EXPECT_TRUE(link1_body->getJoint().getTransformToParentBody().matrix() ==
              T_model_to_world.matrix());
}

TEST(LoadSDFTest, TestDualOffset2) {
  // Loads a one-DOF SDF model with:
  //   1. A Z = 1 and Roll = 90 degree offset between the model's root link's
  //      frame and the model's world frame.
  //   2. A Y = -1 and Roll = -90 degree offset between the model's world frame
  //      and Drake's world frame.
  // They should cancel out resulting in zero offset between the model's world
  // frame and Drake's world frame.
  Eigen::Isometry3d T_model_world_to_drake_world;
  {
    Eigen::Vector3d xyz, rpy;
    xyz << 0, -1, 0;
    rpy << -1.570796326794896557998982, 0, 0;
    T_model_world_to_drake_world.matrix() << rpy2rotmat(rpy), xyz, 0, 0, 0, 1;
  }

  auto weld_to_frame = std::allocate_shared<RigidBodyFrame>(
      Eigen::aligned_allocator<RigidBodyFrame>(), "world",
      nullptr,  // not used since the robot is attached to the world
      T_model_world_to_drake_world);

  RigidBodySystem rbs;
  rbs.addRobotFromFile(
      Drake::getDrakePath() +
          "/systems/plants/test/cylindrical_1dof_robot_offset_z1_r90.sdf",
      DrakeJoint::QUATERNION, weld_to_frame);

  // Verifies that the transform between the robot's root node
  // and the world is equal to identity.
  auto link1_body = rbs.getRigidBodyTree()->findLink("link1");
  EXPECT_TRUE(link1_body != nullptr);
  EXPECT_TRUE(
      link1_body->getJoint().getTransformToParentBody().matrix().isApprox(
          Eigen::Isometry3d::Identity().matrix(), 1e-10))
      << "Incorrect transform from the link1's frame to Drake's world frame."
      << "Got:\n"
      << link1_body->getJoint().getTransformToParentBody().matrix() << "\n"
      << "Expected:\n"
      << Eigen::Isometry3d::Identity().matrix();
}

}  // namespace
}  // namespace test
}  // namespace plants
}  // namespace systems
}  // namespace drake
