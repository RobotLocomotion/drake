#include <string>

#include <gtest/gtest.h>

#include "drake/common/find_resource.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/multibody_plant.h"

namespace drake {
namespace manipulation {
namespace {

// Read the cannonical plant from main sdf model
multibody::ModelInstanceIndex LoadCanonicalModel(
    multibody::MultibodyPlant<double>* plant) {
  const std::string path_to_canonical_model(
      FindResourceOrThrow("drake/manipulation/models/iiwa_description/sdf/"
                          "iiwa14_no_collision.sdf"));
  multibody::Parser parser(plant);
  return parser.AddModelFromFile(path_to_canonical_model);
}

// Tests that KUKA LBR iiwa14 models have consistent values.
// It takes iiwa14_no_collisions.sdf as the cannonical model.
// Note: assumes all joints are declared in the same order.
// TODO(marcoag): when xacro support is used as per (#15613)
// check values on xacro files.
GTEST_TEST(JointLimitsIiwa14, TestEffortVelocityValues) {
  multibody::MultibodyPlant<double> canonical_plant(0.0);
  multibody::ModelInstanceIndex canonical_model_instance =
      LoadCanonicalModel(&canonical_plant);
  canonical_plant.Finalize();

  const std::vector<multibody::JointIndex> joint_canonical_indices =
      canonical_plant.GetJointIndices(canonical_model_instance);

  const std::vector<std::string> path_to_models = {
      "drake/manipulation/models/iiwa_description/sdf/"
      "iiwa14_polytope_collision.sdf",
      "drake/manipulation/models/iiwa_description/urdf/"
      "iiwa14_no_collision.urdf",
      "drake/manipulation/models/iiwa_description/urdf/"
      "iiwa14_polytope_collision.urdf",
      "drake/manipulation/models/iiwa_description/urdf/"
      "iiwa14_primitive_collision.urdf",
      "drake/manipulation/models/iiwa_description/urdf/"
      "iiwa14_spheres_collision.urdf",
      "drake/manipulation/models/iiwa_description/urdf/"
      "iiwa14_spheres_dense_collision.urdf",
      "drake/manipulation/models/iiwa_description/urdf/"
      "iiwa14_spheres_dense_elbow_collision.urdf"
      };

  for (auto& path_to_model : path_to_models) {
    multibody::MultibodyPlant<double> plant(0.0);
    multibody::Parser parser(&plant);
    parser.AddModelFromFile(FindResourceOrThrow(path_to_model));
    plant.Finalize();

    for (int i = 0; i < canonical_plant.num_actuators(); ++i) {
      const multibody::JointActuator<double>& canonical_joint_actuator =
          canonical_plant.get_joint_actuator(
              drake::multibody::JointActuatorIndex(i));
      const multibody::JointActuator<double>& joint_actuator =
          plant.get_joint_actuator(drake::multibody::JointActuatorIndex(i));

      EXPECT_EQ(canonical_joint_actuator.joint().velocity_lower_limits()[0],
                joint_actuator.joint().velocity_lower_limits()[0]);
      EXPECT_EQ(canonical_joint_actuator.joint().velocity_upper_limits()[0],
                joint_actuator.joint().velocity_upper_limits()[0]);
      EXPECT_EQ(canonical_joint_actuator.effort_limit(),
                joint_actuator.effort_limit());
    }
  }
}

// Tests planar KUKA LBR iiwa14 model joint limit values.
// It takes iiwa14_no_collisions.sdf as the cannonical model.
GTEST_TEST(JointLimitsIiwa14, TestEffortVelocityValuesPlanarModel) {
  multibody::MultibodyPlant<double> canonical_plant(0.0);
  multibody::ModelInstanceIndex canonical_model_instance =
      LoadCanonicalModel(&canonical_plant);
  canonical_plant.Finalize();

  const std::vector<multibody::JointIndex> joint_canonical_indices =
      canonical_plant.GetJointIndices(canonical_model_instance);

  multibody::MultibodyPlant<double> plant(0.0);
  multibody::Parser parser(&plant);
  parser.AddModelFromFile(
      FindResourceOrThrow("drake/manipulation/models/iiwa_description/urdf/"
                          "planar_iiwa14_spheres_dense_elbow_collision.urdf"));
  plant.Finalize();

  // This model only has actuators 2, 4 and 6 of the cannonical version
  const multibody::JointActuator<double>& canonical_joint_actuator_2 =
      canonical_plant.get_joint_actuator(
          drake::multibody::JointActuatorIndex(1));
  const multibody::JointActuator<double>& joint_actuator_2 =
      plant.get_joint_actuator(drake::multibody::JointActuatorIndex(0));

  EXPECT_EQ(canonical_joint_actuator_2.joint().velocity_lower_limits()[0],
            joint_actuator_2.joint().velocity_lower_limits()[0]);
  EXPECT_EQ(canonical_joint_actuator_2.joint().velocity_upper_limits()[0],
            joint_actuator_2.joint().velocity_upper_limits()[0]);
  EXPECT_EQ(canonical_joint_actuator_2.effort_limit(),
            joint_actuator_2.effort_limit());

  const multibody::JointActuator<double>& canonical_joint_actuator_4 =
      canonical_plant.get_joint_actuator(
          drake::multibody::JointActuatorIndex(3));
  const multibody::JointActuator<double>& joint_actuator_4 =
      plant.get_joint_actuator(drake::multibody::JointActuatorIndex(1));

  EXPECT_EQ(canonical_joint_actuator_4.joint().velocity_lower_limits()[0],
            joint_actuator_4.joint().velocity_lower_limits()[0]);
  EXPECT_EQ(canonical_joint_actuator_4.joint().velocity_upper_limits()[0],
            joint_actuator_4.joint().velocity_upper_limits()[0]);
  EXPECT_EQ(canonical_joint_actuator_4.effort_limit(),
            joint_actuator_4.effort_limit());

  const multibody::JointActuator<double>& canonical_joint_actuator_6 =
      canonical_plant.get_joint_actuator(
          drake::multibody::JointActuatorIndex(5));
  const multibody::JointActuator<double>& joint_actuator_6 =
      plant.get_joint_actuator(drake::multibody::JointActuatorIndex(2));

  EXPECT_EQ(canonical_joint_actuator_6.joint().velocity_lower_limits()[0],
            joint_actuator_6.joint().velocity_lower_limits()[0]);
  EXPECT_EQ(canonical_joint_actuator_6.joint().velocity_upper_limits()[0],
            joint_actuator_6.joint().velocity_upper_limits()[0]);
  EXPECT_EQ(canonical_joint_actuator_6.effort_limit(),
            joint_actuator_6.effort_limit());
}
}  // namespace
}  // namespace manipulation
}  // namespace drake
