#include <string>

#include <gtest/gtest.h>

#include "drake/common/find_resource.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/multibody_plant.h"

namespace drake {
namespace manipulation {
namespace {

// Read the canonical plant from main URDF model
multibody::ModelInstanceIndex LoadFrankaCanonicalModel(
    multibody::MultibodyPlant<double>* plant) {
  const std::string canonical_model_file(
      FindResourceOrThrow("drake/manipulation/models/franka_description/urdf/"
                          "panda_arm.urdf"));
  multibody::Parser parser(plant);
  return parser.AddModels(canonical_model_file).at(0);
}

// Tests that a model can be loaded into a MultibodyPlant. This unit test also
// verifies that the URDF can be parsed by the URDF parser (similarly for the
// remaining tests in this file).
GTEST_TEST(FrankaArmTest, TestLoadArm) {
  const std::string kPath(
      FindResourceOrThrow("drake/manipulation/models/franka_description/urdf/"
                          "panda_arm.urdf"));

  multibody::MultibodyPlant<double> plant(0.0);
  multibody::Parser parser(&plant);
  parser.AddModels(kPath);

  // There should be actuators for all 7 degrees of freedom.
  EXPECT_EQ(plant.num_actuators(), 7);
  EXPECT_EQ(plant.num_bodies(), 10);
}

GTEST_TEST(FrankaArmTest, TestLoadHand) {
  const std::string kPath(
      FindResourceOrThrow("drake/manipulation/models/franka_description/urdf/"
                          "hand.urdf"));

  multibody::MultibodyPlant<double> plant(0.0);
  multibody::Parser parser(&plant);
  parser.AddModels(kPath);

  EXPECT_EQ(plant.num_actuators(), 2);
  EXPECT_EQ(plant.num_bodies(), 4);
}

GTEST_TEST(FrankaArmTest, TestLoadCombined) {
  const std::string kPath(
      FindResourceOrThrow("drake/manipulation/models/franka_description/urdf/"
                          "panda_arm_hand.urdf"));

  multibody::MultibodyPlant<double> plant(0.0);
  multibody::Parser parser(&plant);
  parser.AddModels(kPath);

  EXPECT_EQ(plant.num_actuators(), 9);
  EXPECT_EQ(plant.num_bodies(), 13);
}

// Tests that the reflected interia values are consistent between the URDFs
GTEST_TEST(FrankaArmTest, TestReflectedInertia) {
  multibody::MultibodyPlant<double> canonical_plant(0.0);
  multibody::ModelInstanceIndex canonical_model_instance =
      LoadFrankaCanonicalModel(&canonical_plant);
  canonical_plant.Finalize();

  const std::vector<multibody::JointIndex> joint_canonical_indices =
      canonical_plant.GetJointIndices(canonical_model_instance);

  const std::string kPath(
      FindResourceOrThrow("drake/manipulation/models/franka_description/urdf/"
                          "panda_arm_hand.urdf"));

  multibody::MultibodyPlant<double> plant(0.0);
  multibody::Parser parser(&plant);
  parser.AddModels(kPath);
  plant.Finalize();

  for (int i = 0; i < canonical_plant.num_actuators(); ++i) {
    const multibody::JointActuator<double>& canonical_joint_actuator =
        canonical_plant.get_joint_actuator(
            drake::multibody::JointActuatorIndex(i));
    const multibody::JointActuator<double>& joint_actuator =
        plant.get_joint_actuator(drake::multibody::JointActuatorIndex(i));

    EXPECT_EQ(canonical_joint_actuator.default_gear_ratio(),
              joint_actuator.default_gear_ratio());
    EXPECT_EQ(canonical_joint_actuator.default_rotor_inertia(),
              joint_actuator.default_rotor_inertia());
  }
}

}  // namespace
}  // namespace manipulation
}  // namespace drake
