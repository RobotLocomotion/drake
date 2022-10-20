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
multibody::ModelInstanceIndex LoadIiwa14CanonicalModel(
    multibody::MultibodyPlant<double>* plant) {
  const std::string canonical_model_file(
      FindResourceOrThrow("drake/manipulation/models/iiwa_description/sdf/"
                          "iiwa14_no_collision.sdf"));
  multibody::Parser parser(plant);
  return parser.AddAllModelsFromFile(canonical_model_file).at(0);
}

// Compares velocity, acceleration, effort and position limits of two given
// actuators.
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
  EXPECT_TRUE(CompareMatrices(joint_a.joint().acceleration_lower_limits(),
            joint_b.joint().acceleration_lower_limits()));
  EXPECT_TRUE(CompareMatrices(joint_a.joint().acceleration_upper_limits(),
            joint_b.joint().acceleration_upper_limits()));
}

// Tests that KUKA LBR iiwa14 models have consistent joint limits.
// It takes iiwa14_no_collisions.sdf as the canonical model.
// It assumes all joints are declared in the same order.
// It checks values directly on urdf files, generated from xacro.
// TODO(marcoag): when xacro support is used as per (#15613)
// check values on files directly generated from xacro,
// rather than ones checked into the source tree with manual edits.
GTEST_TEST(JointLimitsIiwa14, TestEffortVelocityPositionValues) {
  multibody::MultibodyPlant<double> canonical_plant(0.0);
  multibody::ModelInstanceIndex canonical_model_instance =
      LoadIiwa14CanonicalModel(&canonical_plant);
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
      "iiwa14_spheres_dense_elbow_collision.urdf",
      "drake/manipulation/models/iiwa_description/urdf/"
      "dual_iiwa14_polytope_collision.urdf"};

  for (auto& model_file : model_files) {
    multibody::MultibodyPlant<double> plant(0.0);
    multibody::Parser parser(&plant);
    parser.AddAllModelsFromFile(FindResourceOrThrow(model_file));
    plant.Finalize();

    for (int i = 0; i < canonical_plant.num_actuators(); ++i) {
      const multibody::JointActuator<double>& canonical_joint_actuator =
          canonical_plant.get_joint_actuator(
              drake::multibody::JointActuatorIndex(i));
      const multibody::JointActuator<double>& joint_actuator =
          plant.get_joint_actuator(drake::multibody::JointActuatorIndex(i));

      CompareActuatorLimits(canonical_joint_actuator, joint_actuator);

      // Test the joints from the second instance of the dual iiwa14 polytope
      // collision model. They correspond to joints 7 to 13 of the model.
      if (model_file.substr(model_file.find_last_of('/') + 1) ==
          "dual_iiwa14_polytope_collision.urdf") {
        const multibody::JointActuator<double>& second_instance_joint_actuator =
            plant.get_joint_actuator(
                drake::multibody::JointActuatorIndex(i + 7));
        CompareActuatorLimits(canonical_joint_actuator,
                              second_instance_joint_actuator);
      }
    }
  }
}

// Tests planar KUKA LBR iiwa14 model joint limit values.
// It takes iiwa14_no_collisions.sdf as the canonical model.
GTEST_TEST(JointLimitsIiwa14, TestEffortVelocityPositionValuesPlanarModel) {
  multibody::MultibodyPlant<double> canonical_plant(0.0);
  multibody::ModelInstanceIndex canonical_model_instance =
      LoadIiwa14CanonicalModel(&canonical_plant);
  canonical_plant.Finalize();

  const std::vector<multibody::JointIndex> joint_canonical_indices =
      canonical_plant.GetJointIndices(canonical_model_instance);

  multibody::MultibodyPlant<double> plant(0.0);
  multibody::Parser parser(&plant);
  parser.AddAllModelsFromFile(
      FindResourceOrThrow("drake/manipulation/models/iiwa_description/urdf/"
                          "planar_iiwa14_spheres_dense_elbow_collision.urdf"));
  plant.Finalize();

  // This model only has three actuators. They correspond to actuators 1, 3 and
  // 5 from the canonical version.
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
