#include <iostream>

#include <gtest/gtest.h>

#include "drake/math/roll_pitch_yaw.h"
#include "drake/common/drake_path.h"
#include "drake/systems/plants/RigidBodySystem.h"
#include "drake/systems/plants/joints/floating_base_types.h"

namespace drake {
namespace systems {
namespace plants {
namespace {

// Tests the ability to load a URDF as part of the world of a rigid body system.
GTEST_TEST(RigidBodySystemTest, TestLoadURDFWorld) {
  // Instantiates a rigid body system.
  std::unique_ptr<RigidBodySystem> rigid_body_sys(new RigidBodySystem());

  // Adds a URDF to the rigid body system. This URDF contains only fixed joints
  // and is attached to the world via a fixed joint. Thus, everything in the
  // URDF becomes part of the world.
  rigid_body_sys->AddModelInstanceFromFile(
      drake::GetDrakePath() +
          "/systems/plants/test/rigid_body_system/world.urdf",
      drake::systems::plants::joints::kFixed);

  // Verifies that the number of states, inputs, and outputs are all zero.
  EXPECT_EQ(rigid_body_sys->getNumStates(), 0u);
  EXPECT_EQ(rigid_body_sys->getNumInputs(), 0u);
  EXPECT_EQ(rigid_body_sys->getNumOutputs(), 0u);

  // Obtains a const pointer to the rigid body tree within the rigid body
  // system.
  const std::shared_ptr<RigidBodyTree>& tree =
      rigid_body_sys->getRigidBodyTree();

  // Checks that the bodies in the world can be obtained and they have the
  // correct model name.
  for (auto& body_name :
       {"floor", "ramp_1", "ramp_2", "box_1", "box_2", "box_3", "box_4"}) {
    RigidBody* body = tree->FindBody(body_name);
    EXPECT_NE(body, nullptr);
    EXPECT_EQ(body->get_model_name(), "dual_ramps");
  }
}

// Tests the ability to load a SDF multiple times into the same rigid body
// system. The SDF contains more than one model and a sensor on each body.
GTEST_TEST(RigidBodySystemTest, TestLoadSDFMultipleTimes) {
  // Instantiates a rigid body system.
  std::unique_ptr<RigidBodySystem> rigid_body_sys(new RigidBodySystem());

  // Adds the same SDF to the rigid body system twice. Both models are connected
  // to the world using a quaternion joint. The first model's root frame matches
  // the world's coordinate frame. The second model's root frame is offset from
  // the world's coordinate frame by X = 1, Y = 1, and Z = 1.
  rigid_body_sys->AddModelInstanceFromFile(
      drake::GetDrakePath() +
      "/systems/plants/test/rigid_body_system/dual_model_with_sensors.sdf");

  Eigen::Isometry3d T_second_model_to_world;
  {
    Eigen::Vector3d xyz, rpy;
    xyz << 1, 1, 1;
    rpy = Eigen::Vector3d::Zero();
    T_second_model_to_world.matrix()
        << drake::math::rpy2rotmat(rpy), xyz, 0, 0, 0, 1;
  }

  auto weld_to_frame = std::allocate_shared<RigidBodyFrame>(
      Eigen::aligned_allocator<RigidBodyFrame>(), "world", nullptr,
      T_second_model_to_world);

  rigid_body_sys->AddModelInstanceFromFile(
      drake::GetDrakePath() +
          "/systems/plants/test/rigid_body_system/dual_model_with_sensors.sdf",
      drake::systems::plants::joints::kQuaternion, weld_to_frame);

  // Checks that the rigid body system has the correct number of states. The
  // rigid body system has 36 positions + 32 velocities = 68 states.
  EXPECT_EQ(rigid_body_sys->getNumStates(), 68u);

  // Checks that the rigid body system has the correct number of inputs. The
  // rigid body system has 2 joints * 4 models = 8 inputs.
  EXPECT_EQ(rigid_body_sys->getNumInputs(), 8u);

  // Checks that the number of sensors in the rigid body system is correct.
  // The rigid body system has 3 sensors / model * 4 models = 12 sensors.
  EXPECT_EQ(rigid_body_sys->GetSensors().size(), 12u);

  // Checks that the rigid body system has the correct number of outputs. The
  // rigid body system has:
  //   (640 outputs / sensor * 3 sensors / models * 4 models) +
  //   36 position outputs + 32 velocity outputs = 7748 outputs.
  EXPECT_EQ(rigid_body_sys->getNumOutputs(), 7748u);

  // Checks that the rigid body system has the correct number of positions.
  // The rigid body system has:
  //   (7 floating DOFs + 2 joint DOFs) * 4 models = 36 positions.
  EXPECT_EQ(rigid_body_sys->get_num_positions(), 36);

  // Checks that the rigid body system has the correct number of velocities.
  // (6 floating DOFs + 2 joint DOFs) * 4 models = 32 velocities.
  EXPECT_EQ(rigid_body_sys->get_num_velocities(), 32);

  // Obtains a const pointer to the rigid body tree within the rigid body
  // system.
  const std::shared_ptr<RigidBodyTree>& tree =
      rigid_body_sys->getRigidBodyTree();

  // Checks that an exception is thrown if we try to find a body using a
  // non-existent body name.
  EXPECT_THROW(tree->FindBody("non-existent-link"), std::logic_error);

  // Checks that an exception is thrown if we try to find a body using a
  // non-unique body name.
  EXPECT_THROW(tree->FindBody("link_1"), std::logic_error);
  EXPECT_THROW(tree->FindBody("link_2"), std::logic_error);
  EXPECT_THROW(tree->FindBody("link_3"), std::logic_error);

  // Checks that an exception is thrown if we try to find a body using a
  // non-unique body name and model name.
  EXPECT_THROW(tree->FindBody("link_1", "model_1"), std::logic_error);
  EXPECT_THROW(tree->FindBody("link_2", "model_1"), std::logic_error);
  EXPECT_THROW(tree->FindBody("link_3", "model_1"), std::logic_error);
  EXPECT_THROW(tree->FindBody("link_1", "model_2"), std::logic_error);
  EXPECT_THROW(tree->FindBody("link_2", "model_2"), std::logic_error);
  EXPECT_THROW(tree->FindBody("link_3", "model_2"), std::logic_error);

  // Checks that a body can be obtained when we specify the body's name and
  // model instance ID.
  EXPECT_NE(tree->FindBody("link_1", "", 0), nullptr);
  EXPECT_NE(tree->FindBody("link_1", "", 1), nullptr);
  EXPECT_NE(tree->FindBody("link_1", "", 2), nullptr);
  EXPECT_NE(tree->FindBody("link_1", "", 3), nullptr);
  EXPECT_NE(tree->FindBody("link_2", "", 0), nullptr);
  EXPECT_NE(tree->FindBody("link_2", "", 1), nullptr);
  EXPECT_NE(tree->FindBody("link_2", "", 2), nullptr);
  EXPECT_NE(tree->FindBody("link_2", "", 3), nullptr);
  EXPECT_NE(tree->FindBody("link_3", "", 0), nullptr);
  EXPECT_NE(tree->FindBody("link_3", "", 1), nullptr);
  EXPECT_NE(tree->FindBody("link_3", "", 2), nullptr);
  EXPECT_NE(tree->FindBody("link_3", "", 3), nullptr);

  // Checks that a body can be obtained when we specify the body's name, model
  // name, and model instance ID.
  EXPECT_NE(tree->FindBody("link_1", "model_1", 0), nullptr);
  EXPECT_NE(tree->FindBody("link_1", "model_2", 1), nullptr);
  EXPECT_NE(tree->FindBody("link_1", "model_1", 2), nullptr);
  EXPECT_NE(tree->FindBody("link_1", "model_2", 3), nullptr);
  EXPECT_NE(tree->FindBody("link_2", "model_1", 0), nullptr);
  EXPECT_NE(tree->FindBody("link_2", "model_2", 1), nullptr);
  EXPECT_NE(tree->FindBody("link_2", "model_1", 2), nullptr);
  EXPECT_NE(tree->FindBody("link_2", "model_2", 3), nullptr);
  EXPECT_NE(tree->FindBody("link_3", "model_1", 0), nullptr);
  EXPECT_NE(tree->FindBody("link_3", "model_2", 1), nullptr);
  EXPECT_NE(tree->FindBody("link_3", "model_1", 2), nullptr);
  EXPECT_NE(tree->FindBody("link_3", "model_2", 3), nullptr);

  // Checks that we cannot access a non-existent joint.
  EXPECT_THROW(tree->FindChildBodyOfJoint("non-existent-joint"),
      std::runtime_error);

  // Checks that we cannot access a joint using just the joint name.
  // This is impossible because there are multiple joints with the same name.
  EXPECT_THROW(tree->FindChildBodyOfJoint("joint_1"), std::runtime_error);
  EXPECT_THROW(tree->FindChildBodyOfJoint("joint_2"), std::runtime_error);

  // Checks that we can access a joint using the joint name and model instance
  // ID.
  EXPECT_NE(tree->FindChildBodyOfJoint("joint_1", 0), nullptr);
  EXPECT_NE(tree->FindChildBodyOfJoint("joint_1", 1), nullptr);
  EXPECT_NE(tree->FindChildBodyOfJoint("joint_1", 2), nullptr);
  EXPECT_NE(tree->FindChildBodyOfJoint("joint_1", 3), nullptr);
  EXPECT_NE(tree->FindChildBodyOfJoint("joint_2", 0), nullptr);
  EXPECT_NE(tree->FindChildBodyOfJoint("joint_2", 1), nullptr);
  EXPECT_NE(tree->FindChildBodyOfJoint("joint_2", 2), nullptr);
  EXPECT_NE(tree->FindChildBodyOfJoint("joint_2", 3), nullptr);
}

// Tests whether the URDF parser is robust against an improperly specified
// transmission element. In this case, the transmission element specifies a
// non-existent joint. For more information, see:
// https://github.com/RobotLocomotion/drake/issues/1864
GTEST_TEST(RigidBodySystemTest, TestLoadURDFWithBadTransmission) {
  // Instantiates a rigid body system.
  std::unique_ptr<RigidBodySystem> rigid_body_sys(new RigidBodySystem());

  // Tests whether an exception is properly thrown when loading a URDF with a
  // transmission that specifies a non-existent joint. This is done by first
  // verifying that an exception is thrown, and then verifying that the thrown
  // exception is the correct one.
  EXPECT_THROW(
      rigid_body_sys->AddModelInstanceFromFile(drake::GetDrakePath() +
                                       "/systems/plants/test/rigid_body_system/"
                                       "bad_transmission_no_joint.urdf"),
      std::runtime_error);

  try {
    rigid_body_sys->AddModelInstanceFromFile(drake::GetDrakePath() +
                                     "/systems/plants/test/rigid_body_system/"
                                     "bad_transmission_no_joint.urdf");
  } catch (std::runtime_error& error) {
    // Asserts that the exception is thrown when FindBodyOfJoint() fails to find
    // find a non-existing joint.
    EXPECT_TRUE(std::string(error.what()).find("FindChildBodyOfJoint") !=
                std::string::npos);
  }
}

}  // namespace
}  // namespace plants
}  // namespace systems
}  // namespace drake
