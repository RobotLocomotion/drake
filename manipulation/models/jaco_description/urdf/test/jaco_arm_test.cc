#include <string>

#include <gtest/gtest.h>

#include "drake/common/find_resource.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/multibody_plant.h"

namespace drake {
namespace manipulation {
namespace {

// Tests that j2n6s300.urdf can be loaded into a RigidBodyTree. This
// unit test also verifies that the URDF can be parsed by the URDF
// parser.
GTEST_TEST(JacoArmTest, TestLoad6DofTree) {
  const std::string kPath(FindResourceOrThrow(
      "drake/manipulation/models/jaco_description/urdf/"
      "j2n6s300.urdf"));

  multibody::MultibodyPlant<double> plant;
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

// Tests that j2s7s300.urdf can be loaded into a RigidBodyTree. This
// unit test also verifies that the URDF can be parsed by the URDF
// parser.
GTEST_TEST(JacoArmTest, TestLoad7DofTree) {
  const std::string kPath(FindResourceOrThrow(
      "drake/manipulation/models/jaco_description/urdf/"
      "j2s7s300.urdf"));

  multibody::MultibodyPlant<double> plant;
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

}  // namespace
}  // namespace manipulation
}  // namespace drake
