#include <iostream>

#include <gtest/gtest.h>

#include "drake/Path.h"
#include "drake/systems/plants/RigidBodySystem.h"

namespace drake {
namespace systems {
namespace plants {
namespace test {
namespace rigid_body_system {
namespace {

using Drake::RigidBodySystem;

GTEST_TEST(RigidBodySystemTest, TestLoadSDFMultipleTimes) {
  // Instantiates a rigid body system.
  std::unique_ptr<RigidBodySystem> rigid_body_sys(new RigidBodySystem());

  // Adds the same SDF to the rigid body system twice. Both models are connected
  // to the world using a quaternion joint. The first model's root frame match
  // the world's coordinate frame. The second model's root frame is offset from
  // the world's coordinate fram by X = 1, Y = 1, and Z = 1.
  rigid_body_sys->addRobotFromFile(Drake::getDrakePath() +
    "/systems/plants/test/rigid_body_system/dual_model_with_sensors.sdf");

  Eigen::Isometry3d T_second_model_to_world;
  {
    Eigen::Vector3d xyz, rpy;
    xyz << 1, 1, 1;
    rpy = Eigen::Vector3d::Zero();
    T_second_model_to_world.matrix() << rpy2rotmat(rpy), xyz, 0, 0, 0, 1;
  }

  auto weld_to_frame = std::allocate_shared<RigidBodyFrame>(
      Eigen::aligned_allocator<RigidBodyFrame>(), "world", nullptr,
      T_second_model_to_world);

  rigid_body_sys->addRobotFromFile(Drake::getDrakePath() +
    "/systems/plants/test/rigid_body_system/dual_model_with_sensors.sdf",
    DrakeJoint::QUATERNION, weld_to_frame);

  // Checks that the rigid body system has the correct number of states. The
  // rigid body system has 36 positions + 32 velocities = 68 states.
  EXPECT_EQ(rigid_body_sys->getNumStates(), 68);

  // Checks that the rigid body system has the correct number of inputs. The
  // rigid body system has 2 joints * 4 robots = 8 inputs.
  EXPECT_EQ(rigid_body_sys->getNumInputs(), 8);

  // Checks that the number of sensors in the rigid body system is correct.
  // The rigid body system has 3 sensors / model * 4 models = 12 sensors.
  EXPECT_EQ(rigid_body_sys->GetSensors().size(), 12);

  // Checks that the rigid body system has the correct number of outputs. The
  // rigid body system has:
  //   (640 outputs / sensor * 3 sensors / models * 4 models) +
  //   36 position outputs + 32 velocity outputs = 7748.
  EXPECT_EQ(rigid_body_sys->getNumOutputs(), 7748);

  // Checks that the rigid body system has the correct number of positions.
  // (7 floating DOFs + 2 joint DOFs) * 4 models = 36
  EXPECT_EQ(rigid_body_sys->number_of_positions(), 36);

  // Checks that the rigid body system has the correct number of velocities.
  // (6 floating DOFs + 2 joint DOFs) * 4 models = 32
  EXPECT_EQ(rigid_body_sys->number_of_velocities(), 32);

  const std::shared_ptr<RigidBodyTree>& tree =
    rigid_body_sys->getRigidBodyTree();

  // Checks that an exception is thrown if we try to find a link using a
  // non-existent link name.
  EXPECT_THROW(tree->findLink("non-existent-link"), std::logic_error);

  // Checks that an exception is thrown if we try to find a link using a
  // non-unique link name.
  EXPECT_THROW(tree->findLink("link_1"), std::logic_error);

  // Checks that an exception is thrown if we try to find a link using a
  // non-unique link name and model name.
  EXPECT_THROW(tree->findLink("link_1", "model_1"), std::logic_error);
  EXPECT_THROW(tree->findLink("link_2", "model_1"), std::logic_error);
  EXPECT_THROW(tree->findLink("link_3", "model_1"), std::logic_error);
  EXPECT_THROW(tree->findLink("link_1", "model_2"), std::logic_error);
  EXPECT_THROW(tree->findLink("link_2", "model_2"), std::logic_error);
  EXPECT_THROW(tree->findLink("link_3", "model_2"), std::logic_error);

  // Checks that a link can be obtained when we specify the link's name and
  // model ID.
  EXPECT_NE(tree->findLink("link_1", "", 0), nullptr);
  EXPECT_NE(tree->findLink("link_1", "", 1), nullptr);
  EXPECT_NE(tree->findLink("link_1", "", 2), nullptr);
  EXPECT_NE(tree->findLink("link_1", "", 3), nullptr);

  // Checks that a link can be obtained when we specify the link's name, model
  // name, and model ID.
  EXPECT_NE(tree->findLink("link_1", "model_1", 0), nullptr);
  EXPECT_NE(tree->findLink("link_1", "model_2", 1), nullptr);
  EXPECT_NE(tree->findLink("link_1", "model_1", 2), nullptr);
  EXPECT_NE(tree->findLink("link_1", "model_2", 3), nullptr);

  // Checks that we cannot access a non-existent joint.
  EXPECT_THROW(tree->findJoint("non-existent-joint"), std::logic_error);

  // Checks that we cannot access a joint using just the joint name.
  EXPECT_THROW(tree->findJoint("joint_1"), std::logic_error);
  EXPECT_THROW(tree->findJoint("joint_2"), std::logic_error);

  // Checks that we can access a joint using the joint name and model ID.
  EXPECT_NE(tree->findJoint("joint_1", 0), nullptr);
  EXPECT_NE(tree->findJoint("joint_1", 1), nullptr);
  EXPECT_NE(tree->findJoint("joint_1", 2), nullptr);
  EXPECT_NE(tree->findJoint("joint_1", 3), nullptr);
  EXPECT_NE(tree->findJoint("joint_2", 0), nullptr);
  EXPECT_NE(tree->findJoint("joint_2", 1), nullptr);
  EXPECT_NE(tree->findJoint("joint_2", 2), nullptr);
  EXPECT_NE(tree->findJoint("joint_2", 3), nullptr);
}

}  // namespace
}  // namespace rigid_body_system
}  // namespace test
}  // namespace plants
}  // namespace systems
}  // namespace drake
