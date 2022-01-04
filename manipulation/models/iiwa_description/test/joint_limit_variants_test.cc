#include <string>

#include <gtest/gtest.h>

#include "drake/common/find_resource.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/multibody_plant.h"

namespace drake {
namespace manipulation {
namespace {

// Read the canonical plant from main SDFormat model
multibody::ModelInstanceIndex LoadCanonicalModel(
    multibody::MultibodyPlant<double>* plant) {
  const std::string canonical_model_file(
      FindResourceOrThrow("drake/manipulation/models/iiwa_description/sdf/"
                          "iiwa14_no_collision.sdf"));
  multibody::Parser parser(plant);
  return parser.AddModelFromFile(canonical_model_file);
}

// Compares velocity, effort and position limits of two given actuators
void CompareActuatorLimits(const multibody::JointActuator<double>& joint_a,
                           const multibody::JointActuator<double>& joint_b) {
  EXPECT_NE(&joint_a, &joint_b);  // Different instance.
  EXPECT_TRUE(CompareMatrices(joint_a.joint().velocity_lower_limits(),
                              joint_b.joint().velocity_lower_limits()));
  EXPECT_TRUE(CompareMatrices(joint_a.joint().velocity_upper_limits(),
                              joint_b.joint().velocity_upper_limits()));
  EXPECT_TRUE(CompareMatrices(joint_a.joint().position_lower_limits(),
                              joint_b.joint().position_lower_limits()));
  EXPECT_TRUE(CompareMatrices(joint_a.joint().position_upper_limits(),
                              joint_b.joint().position_upper_limits()));
  EXPECT_EQ(joint_a.effort_limit(), joint_b.effort_limit());
}

// Tests that KUKA LBR iiwa14 models have consistent joint limits.
// It takes iiwa14_no_collisions.sdf as the canonical model.
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

  const std::vector<std::string> model_files = {
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
      "iiwa14_spheres_dense_elbow_collision.urdf"};

  for (auto& model_file : model_files) {
    multibody::MultibodyPlant<double> plant(0.0);
    multibody::Parser parser(&plant);
    parser.AddModelFromFile(FindResourceOrThrow(model_file));
    plant.Finalize();

    for (int i = 0; i < canonical_plant.num_actuators(); ++i) {
      const multibody::JointActuator<double>& canonical_joint_actuator =
          canonical_plant.get_joint_actuator(
              drake::multibody::JointActuatorIndex(i));
      const multibody::JointActuator<double>& joint_actuator =
          plant.get_joint_actuator(drake::multibody::JointActuatorIndex(i));

      CompareActuatorLimits(canonical_joint_actuator, joint_actuator);
    }
  }
}

// Tests planar KUKA LBR iiwa14 model joint limit values.
// It takes iiwa14_no_collisions.sdf as the canonical model.
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

  // This model only has actuators 2, 4 and 6 of the canonical version
  CompareActuatorLimits(
      canonical_plant.get_joint_actuator(
          drake::multibody::JointActuatorIndex(1)),
      plant.get_joint_actuator(drake::multibody::JointActuatorIndex(0)));
  CompareActuatorLimits(
      canonical_plant.get_joint_actuator(
          drake::multibody::JointActuatorIndex(3)),
      plant.get_joint_actuator(drake::multibody::JointActuatorIndex(1)));
  CompareActuatorLimits(
      canonical_plant.get_joint_actuator(
          drake::multibody::JointActuatorIndex(5)),
      plant.get_joint_actuator(drake::multibody::JointActuatorIndex(2)));
}
}  // namespace
}  // namespace manipulation
}  // namespace drake
