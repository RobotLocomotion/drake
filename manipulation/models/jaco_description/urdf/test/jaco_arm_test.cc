#include <string>

#include <gtest/gtest.h>

#include "drake/common/find_resource.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/multibody_plant.h"

namespace drake {
namespace manipulation {
namespace {

// Tests that j2n6s300.urdf can be loaded into a MultibodyPlant. This unit
// test also verifies that the URDF can be parsed by the URDF parser.
GTEST_TEST(JacoArmTest, TestLoad6DofTree) {
  const std::string kPath(FindResourceOrThrow(
      "drake/manipulation/models/jaco_description/urdf/"
      "j2n6s300.urdf"));

  multibody::MultibodyPlant<double> plant(0.0);
  multibody::Parser parser(&plant);
  parser.AddModelFromFile(kPath);

  // There should be actuators for all 6 degrees of freedom and 3
  // fingers.
  EXPECT_EQ(plant.num_actuators(), 9);

  // Each robot has 6 bodies, an end effector, and with 6 finger
  // bodies.  In addition, there are three bodies that are not part of
  // the robot: world, root, and base. Hence, there should be a total
  // of 6 + 1 + 6 + 3 = 16 bodies in the tree.
  EXPECT_EQ(plant.num_bodies(), 16);
}

// Tests that the urdf can be loaded into a MultibodyPlant. This unit
// test also verifies that the URDF can be parsed by the URDF parser.
GTEST_TEST(JacoArmTest, TestLoad7DofTree) {
  const std::string kPath(FindResourceOrThrow(
      "drake/manipulation/models/jaco_description/urdf/"
      "j2s7s300.urdf"));

  multibody::MultibodyPlant<double> plant(0.0);
  multibody::Parser parser(&plant);
  parser.AddModelFromFile(kPath);

  // There should be actuators for all 7 degrees of freedom and 3
  // fingers.
  EXPECT_EQ(plant.num_actuators(), 10);

  // Each robot has 7 bodies, an end effector, and with 6 finger
  // bodies.  In addition, there are three bodies that are not part of
  // the robot: world, root, and base. Hence, there should be a total
  // of 7 + 1 + 6 + 3 = 16 bodies in the tree.
  EXPECT_EQ(plant.num_bodies(), 17);
}

// Tests that the urdf can be loaded into a MultibodyPlant. This unit
// test also verifies that the URDF can be parsed by the URDF parser.
GTEST_TEST(JacoArmTest, TestLoad7DofSphereCollisionTree) {
  const std::string kPath(FindResourceOrThrow(
      "drake/manipulation/models/jaco_description/urdf/"
      "j2s7s300_sphere_collision.urdf"));

  multibody::MultibodyPlant<double> plant(0.0);
  multibody::Parser parser(&plant);
  parser.AddModelFromFile(kPath);

  // There should be actuators for all 7 degrees of freedom and 3
  // fingers.
  EXPECT_EQ(plant.num_actuators(), 10);

  // Each robot has 7 bodies, an end effector, and with 6 finger
  // bodies.  In addition, there are three bodies that are not part of
  // the robot: world, root, and base. Hence, there should be a total
  // of 7 + 1 + 6 + 3 = 16 bodies in the tree.
  EXPECT_EQ(plant.num_bodies(), 17);
}

// Tests that the urdf can be loaded into a MultibodyPlant. This unit
// test also verifies that the URDF can be parsed by the URDF parser.
GTEST_TEST(JacoArmTest, TestLoad7DofTreeArmOnly) {
  const std::string kPath(FindResourceOrThrow(
      "drake/manipulation/models/jaco_description/urdf/"
      "j2s7s300_arm.urdf"));

  multibody::MultibodyPlant<double> plant(0.0);
  multibody::Parser parser(&plant);
  parser.AddModelFromFile(kPath);

  EXPECT_EQ(plant.num_actuators(), 7);

  // Each arm has 7 bodies. In addition, there are three bodies that are not
  // part of the robot: world, root, and base. Hence, there should be a total
  // of 7 + 3 = 10 bodies in the tree.
  EXPECT_EQ(plant.num_bodies(), 10);
}

// Tests that the urdf can be loaded into a MultibodyPlant. This unit
// test also verifies that the URDF can be parsed by the URDF parser.
GTEST_TEST(JacoArmTest, TestLoad7DofTreeSphereCollisionArmOnly) {
  const std::string kPath(FindResourceOrThrow(
      "drake/manipulation/models/jaco_description/urdf/"
      "j2s7s300_arm_sphere_collision.urdf"));

  multibody::MultibodyPlant<double> plant(0.0);
  multibody::Parser parser(&plant);
  parser.AddModelFromFile(kPath);

  EXPECT_EQ(plant.num_actuators(), 7);

  // Each arm has 7 bodies. In addition, there are three bodies that are not
  // part of the robot: world, root, and base. Hence, there should be a total
  // of 7 + 3 = 10 bodies in the tree.
  EXPECT_EQ(plant.num_bodies(), 10);
}

// Tests that the urdf can be loaded into a MultibodyPlant. This unit
// test also verifies that the URDF can be parsed by the URDF parser.
GTEST_TEST(JacoArmTest, TestLoad7DofTreeHandOnly) {
  const std::string kPath(FindResourceOrThrow(
      "drake/manipulation/models/jaco_description/urdf/"
      "j2s7s300_hand.urdf"));

  multibody::MultibodyPlant<double> plant(0.0);
  multibody::Parser parser(&plant);
  parser.AddModelFromFile(kPath);

  // There should be actuators for all 3 fingers.
  EXPECT_EQ(plant.num_actuators(), 3);

  // Each hand has a base, an end effector, and 6 finger bodies.  In addition,
  // there is one world body.  Hence, there should be a total of 1 + 6 + 2 =
  // 9 bodies in the tree.
  EXPECT_EQ(plant.num_bodies(), 9);
}

// Tests that the urdf can be loaded into a MultibodyPlant. This unit
// test also verifies that the URDF can be parsed by the URDF parser.
GTEST_TEST(JacoArmTest, TestLoad7DofTreeSphereCollisionHandOnly) {
  const std::string kPath(FindResourceOrThrow(
      "drake/manipulation/models/jaco_description/urdf/"
      "j2s7s300_hand_sphere_collision.urdf"));

  multibody::MultibodyPlant<double> plant(0.0);
  multibody::Parser parser(&plant);
  parser.AddModelFromFile(kPath);

  // There should be actuators for all 3 fingers.
  EXPECT_EQ(plant.num_actuators(), 3);

  // Each hand has a base, an end effector, and 6 finger bodies.  In addition,
  // there is one world body.  Hence, there should be a total of 1 + 6 + 2 =
  // 9 bodies in the tree.
  EXPECT_EQ(plant.num_bodies(), 9);
}

}  // namespace
}  // namespace manipulation
}  // namespace drake
