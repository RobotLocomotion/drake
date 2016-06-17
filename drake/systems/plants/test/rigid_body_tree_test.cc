#include <iostream>

#include <gtest/gtest.h>

#include "drake/systems/plants/RigidBodyTree.h"

namespace drake {
namespace systems {
namespace plants {
namespace test {
namespace {

class RigidBodyTreeTest : public ::testing::Test {
 protected:
  virtual void SetUp() {
    tree.reset(new RigidBodyTree());

    // Defines four rigid bodies.
    r1b1 = new RigidBody();
    r1b1->model_name_ = "robot1";
    r1b1->name_ = "body1";

    r2b1 = new RigidBody();
    r2b1->model_name_ = "robot2";
    r2b1->name_ = "body1";

    r3b1 = new RigidBody();
    r3b1->model_name_ = "robot3";
    r3b1->name_ = "body1";

    r4b1 = new RigidBody();
    r4b1->model_name_ = "robot4";
    r4b1->name_ = "body1";
  }

 public:
  // TODO(amcastro-tri): A stack object here (preferable to a pointer)
  // generates build issues on Windows platforms. See git-hub issue #1854.
  std::unique_ptr<RigidBodyTree> tree;
  // TODO(amcastro-tri): these pointers will be replaced by Sherm's
  // unique_ptr_reference's.
  RigidBody* r1b1{};
  RigidBody* r2b1{};
  RigidBody* r3b1{};
  RigidBody* r4b1{};
};

TEST_F(RigidBodyTreeTest, TestAddFloatingJointNoOffset) {
  // Adds rigid bodies r1b1 and r2b1 to the rigid body tree and verify they can
  // be found.

  // RigidBodyTree takes ownership of these bodies.
  // User still has access to these bodies through the raw pointers.
  tree->add_rigid_body(std::unique_ptr<RigidBody>(r1b1));
  tree->add_rigid_body(std::unique_ptr<RigidBody>(r2b1));

  EXPECT_TRUE(tree->findLink("body1", "robot1") != nullptr);
  EXPECT_TRUE(tree->findLink("body1", "robot2") != nullptr);
  EXPECT_THROW(tree->findLink("body2", "robot1"), std::logic_error);
  EXPECT_THROW(tree->findLink("body2", "robot2"), std::logic_error);

  // Adds floating joints that connect r1b1 and r2b1 to the rigid body tree's
  // world link at zero offset.
  tree->AddFloatingJoint(DrakeJoint::QUATERNION,
                        {r1b1->body_index, r2b1->body_index});

  // Verfies that the two rigid bodies are located in the correct place.
  const DrakeJoint& jointR1B1 = tree->findLink("body1", "robot1")->getJoint();
  EXPECT_TRUE(jointR1B1.isFloating());
  EXPECT_TRUE(jointR1B1.getTransformToParentBody().matrix() ==
              Eigen::Isometry3d::Identity().matrix());

  const DrakeJoint& jointR2B1 = tree->findLink("body1", "robot2")->getJoint();
  EXPECT_TRUE(jointR2B1.isFloating());
  EXPECT_TRUE(jointR2B1.getTransformToParentBody().matrix() ==
              Eigen::Isometry3d::Identity().matrix());
}

TEST_F(RigidBodyTreeTest, TestAddFloatingJointWithOffset) {
  // TODO(amcastro-tri): these pointers will be replaced by Sherm's
  // unique_ptr_reference's
  tree->add_rigid_body(std::unique_ptr<RigidBody>(r1b1));
  tree->add_rigid_body(std::unique_ptr<RigidBody>(r2b1));

  // Adds floating joints that connect r1b1 and r2b1 to the rigid body tree's
  // world link at offset x = 1, y = 1, z = 1.
  Eigen::Isometry3d T_r1and2_to_world;
  {
    Eigen::Vector3d xyz, rpy;
    xyz << 1, 1, 1;
    rpy = Eigen::Vector3d::Zero();
    T_r1and2_to_world.matrix() << rpy2rotmat(rpy), xyz, 0, 0, 0, 1;
  }

  auto weld_to_frame = std::allocate_shared<RigidBodyFrame>(
      Eigen::aligned_allocator<RigidBodyFrame>(), "world", nullptr,
      T_r1and2_to_world);

  tree->AddFloatingJoint(DrakeJoint::QUATERNION,
                        {r1b1->body_index, r2b1->body_index}, weld_to_frame);

  // Verfies that the two rigid bodies are located in the correct place.
  const DrakeJoint& jointR1B1 = tree->findLink("body1", "robot1")->getJoint();
  EXPECT_TRUE(jointR1B1.isFloating());
  EXPECT_TRUE(jointR1B1.getTransformToParentBody().matrix() ==
              T_r1and2_to_world.matrix());

  const DrakeJoint& jointR2B1 = tree->findLink("body1", "robot2")->getJoint();
  EXPECT_TRUE(jointR2B1.isFloating());
  EXPECT_TRUE(jointR2B1.getTransformToParentBody().matrix() ==
              T_r1and2_to_world.matrix());
}

TEST_F(RigidBodyTreeTest, TestAddFloatingJointWeldToLink) {
  // Adds rigid body r1b1 to the rigid body tree and welds it to the world with
  // zero offset. Verifies that it is in the correct place.
  // TODO(amcastro-tri): these pointers will be replaced by Sherm's
  // unique_ptr_reference's
  tree->add_rigid_body(std::unique_ptr<RigidBody>(r1b1));

  tree->AddFloatingJoint(DrakeJoint::QUATERNION, {r1b1->body_index});

  // Adds rigid body r2b1 to the rigid body tree and welds it to r1b1 with
  // offset x = 1, y = 1, z = 1. Verifies that it is in the correct place.
  // TODO(amcastro-tri): these pointers will be replaced by Sherm's
  // unique_ptr_reference's
  tree->add_rigid_body(std::unique_ptr<RigidBody>(r2b1));

  Eigen::Isometry3d T_r2_to_r1;
  {
    Eigen::Vector3d xyz, rpy;
    xyz << 1, 1, 1;
    rpy = Eigen::Vector3d::Zero();
    T_r2_to_r1.matrix() << rpy2rotmat(rpy), xyz, 0, 0, 0, 1;
  }

  auto r2b1_weld = std::allocate_shared<RigidBodyFrame>(
      Eigen::aligned_allocator<RigidBodyFrame>(), "body1",
      tree->findLink("body1", "robot1"), T_r2_to_r1);

  tree->AddFloatingJoint(DrakeJoint::QUATERNION, {r2b1->body_index}, r2b1_weld);

  // Adds rigid body r3b1 and r4b1 to the rigid body tree and welds it to r2b1
  // with offset x = 2, y = 2, z = 2. Verifies that it is in the correct place.
  // TODO(amcastro-tri): these pointers will be replaced by Sherm's
  // unique_ptr_reference's
  tree->add_rigid_body(std::unique_ptr<RigidBody>(r3b1));
  tree->add_rigid_body(std::unique_ptr<RigidBody>(r4b1));

  Eigen::Isometry3d T_r3_and_r4_to_r2;
  {
    Eigen::Vector3d xyz, rpy;
    xyz << 2, 2, 2;
    rpy = Eigen::Vector3d::Zero();
    T_r3_and_r4_to_r2.matrix() << rpy2rotmat(rpy), xyz, 0, 0, 0, 1;
  }

  auto r3b1_and_r4b1_weld = std::allocate_shared<RigidBodyFrame>(
      Eigen::aligned_allocator<RigidBodyFrame>(), "body1",
      tree->findLink("body1", "robot2"), T_r3_and_r4_to_r2);

  tree->AddFloatingJoint(DrakeJoint::QUATERNION,
                        {r3b1->body_index, r4b1->body_index},
                        r3b1_and_r4b1_weld);

  EXPECT_TRUE(tree->findLink("body1", "robot1")
                  ->getJoint()
                  .getTransformToParentBody()
                  .matrix() == Eigen::Isometry3d::Identity().matrix());

  EXPECT_TRUE(tree->findLink("body1", "robot2")
                  ->getJoint()
                  .getTransformToParentBody()
                  .matrix() == T_r2_to_r1.matrix());

  EXPECT_TRUE(tree->findLink("body1", "robot3")
                  ->getJoint()
                  .getTransformToParentBody()
                  .matrix() == T_r3_and_r4_to_r2.matrix());

  EXPECT_TRUE(tree->findLink("body1", "robot4")
                  ->getJoint()
                  .getTransformToParentBody()
                  .matrix() == T_r3_and_r4_to_r2.matrix());
}

}  // namespace
}  // namespace test
}  // namespace plants
}  // namespace systems
}  // namespace drake
