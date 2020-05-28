#include <string>

#include <gtest/gtest.h>

#include "drake/common/find_resource.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/multibody_plant.h"

namespace drake {
namespace manipulation {
namespace {

// Tests that a model can be loaded into a MultibodyPlant. This unit test also
// verifies that the URDF can be parsed by the URDF parser (similarly for the
// remaining tests in this file).
GTEST_TEST(FrankaArmTest, TestLoadArm) {
  const std::string kPath(FindResourceOrThrow(
      "drake/manipulation/models/franka_description/urdf/"
      "panda_arm.urdf"));

  multibody::MultibodyPlant<double> plant(0.0);
  multibody::Parser parser(&plant);
  parser.AddModelFromFile(kPath);

  // There should be actuators for all 7 degrees of freedom.
  EXPECT_EQ(plant.num_actuators(), 7);
  EXPECT_EQ(plant.num_bodies(), 10);
}

GTEST_TEST(FrankaArmTest, TestLoadHand) {
  const std::string kPath(FindResourceOrThrow(
      "drake/manipulation/models/franka_description/urdf/"
      "hand.urdf"));

  multibody::MultibodyPlant<double> plant(0.0);
  multibody::Parser parser(&plant);
  parser.AddModelFromFile(kPath);

  EXPECT_EQ(plant.num_actuators(), 2);
  EXPECT_EQ(plant.num_bodies(), 4);
}

GTEST_TEST(FrankaArmTest, TestLoadCombined) {
  const std::string kPath(FindResourceOrThrow(
      "drake/manipulation/models/franka_description/urdf/"
      "panda_arm_hand.urdf"));

  multibody::MultibodyPlant<double> plant(0.0);
  multibody::Parser parser(&plant);
  parser.AddModelFromFile(kPath);

  EXPECT_EQ(plant.num_actuators(), 9);
  EXPECT_EQ(plant.num_bodies(), 13);
}


}  // namespace
}  // namespace manipulation
}  // namespace drake
