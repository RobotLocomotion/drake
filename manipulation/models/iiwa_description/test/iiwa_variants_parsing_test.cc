#include <string>

#include <gtest/gtest.h>

#include "drake/common/find_resource.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/multibody_plant.h"

namespace drake {
namespace manipulation {
namespace {

// Read the canonical plant for IIWA7 from main SDFormat model
multibody::ModelInstanceIndex LoadIiwa7CanonicalModel(
    multibody::MultibodyPlant<double>* plant) {
  const std::string canonical_model_file(
      FindResourceOrThrow("drake/manipulation/models/iiwa_description/iiwa7/"
                          "iiwa7_no_collision.sdf"));
  multibody::Parser parser(plant);
  return parser.AddModels(canonical_model_file).at(0);
}
// Read the canonical plant for IIWA14 from main SDFormat model
multibody::ModelInstanceIndex LoadIiwa14CanonicalModel(
    multibody::MultibodyPlant<double>* plant) {
  const std::string canonical_model_file(
      FindResourceOrThrow("drake/manipulation/models/iiwa_description/sdf/"
                          "iiwa14_no_collision.sdf"));
  multibody::Parser parser(plant);
  return parser.AddModels(canonical_model_file).at(0);
}

// Read the common robot model files for IIWA14
const std::vector<std::string> GetCommonIiwa14ModelFiles() {
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
  return model_files;
}

// Read the common robot model files for IIWA7
const std::vector<std::string> GetCommonIiwa7ModelFiles() {
  const std::vector<std::string> model_files = {
      "drake/manipulation/models/iiwa_description/iiwa7/"
      "iiwa7_no_collision.sdf",
      "drake/manipulation/models/iiwa_description/iiwa7/"
      "iiwa7_with_box_collision.sdf"};
  return model_files;
}

// Read the dual IIWA model file
const std::string GetDualIiwaModelFile() {
  return "drake/manipulation/models/iiwa_description/urdf/"
         "dual_iiwa14_polytope_collision.urdf";
}

// Read the planar IIWA model file
const std::string GetPlanarIiwaModelFile() {
  return "drake/manipulation/models/iiwa_description/urdf/"
         "planar_iiwa14_spheres_dense_elbow_collision.urdf";
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
  EXPECT_EQ(joint_a.default_gear_ratio(), joint_b.default_gear_ratio());
  EXPECT_EQ(joint_a.default_rotor_inertia(), joint_b.default_rotor_inertia());
}

// Compare rotational inertias i.e moments and products of inertia
void CompareRotationalInertias(
    const multibody::RigidBody<double>& canonical_body,
    const multibody::RigidBody<double>& robot_body) {
  EXPECT_TRUE(
      CompareMatrices(canonical_body.default_rotational_inertia().get_moments(),
                      robot_body.default_rotational_inertia().get_moments()));
  EXPECT_TRUE(CompareMatrices(
      canonical_body.default_rotational_inertia().get_products(),
      robot_body.default_rotational_inertia().get_products()));
}

// Tests that KUKA LBR iiwa7 models have consistent joint limits.
// It takes iiwa7_no_collision.sdf as the canonical model.
// It assumes all joints are declared in the same order.
GTEST_TEST(JointLimitsIiwa7, TestEffortVelocityPositionValues) {
  multibody::MultibodyPlant<double> canonical_plant(0.0);
  multibody::ModelInstanceIndex canonical_model_instance =
      LoadIiwa7CanonicalModel(&canonical_plant);
  canonical_plant.Finalize();

  const std::vector<multibody::JointIndex> joint_canonical_indices =
      canonical_plant.GetJointIndices(canonical_model_instance);

  std::vector<std::string> model_files = GetCommonIiwa7ModelFiles();

  for (auto& model_file : model_files) {
    SCOPED_TRACE(fmt::format("model file: {}", model_file));
    multibody::MultibodyPlant<double> plant(0.0);
    multibody::Parser parser(&plant);
    parser.AddModels(FindResourceOrThrow(model_file));
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

// Tests that KUKA LBR iiwa7 models have consistent inertias.
// It takes iiwa7_no_collision.sdf as the canonical model.
// It assumes all links are declared in the same order.
// It checks values directly on urdf files, generated from xacro.
GTEST_TEST(InertiasIiwa7, TestInertiaValues) {
  multibody::MultibodyPlant<double> canonical_plant(0.0);
  multibody::ModelInstanceIndex canonical_model_instance =
      LoadIiwa7CanonicalModel(&canonical_plant);
  canonical_plant.Finalize();

  const std::vector<multibody::BodyIndex> body_canonical_indices =
      canonical_plant.GetBodyIndices(canonical_model_instance);

  std::vector<std::string> model_files = GetCommonIiwa7ModelFiles();

  std::vector<std::string> link_names = {
      "iiwa_link_0", "iiwa_link_1", "iiwa_link_2", "iiwa_link_3",
      "iiwa_link_4", "iiwa_link_5", "iiwa_link_6", "iiwa_link_7"};

  for (auto& model_file : model_files) {
    SCOPED_TRACE(fmt::format("model file: {}", model_file));
    multibody::MultibodyPlant<double> plant(0.0);
    multibody::Parser parser(&plant);
    parser.AddModels(FindResourceOrThrow(model_file));
    plant.Finalize();
    std::filesystem::path model_path(model_file);
    for (size_t i = 0; i < link_names.size(); ++i) {
      SCOPED_TRACE(fmt::format("Link: {}", link_names[i]));
      const multibody::RigidBody<double>& canonical_body =
          canonical_plant.GetBodyByName(link_names[i]);
      const multibody::RigidBody<double>& robot_body =
          plant.GetBodyByName(link_names[i]);
      CompareRotationalInertias(canonical_body, robot_body);
    }
  }
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

  std::vector<std::string> model_files = GetCommonIiwa14ModelFiles();
  model_files.push_back(GetDualIiwaModelFile());

  for (auto& model_file : model_files) {
    multibody::MultibodyPlant<double> plant(0.0);
    multibody::Parser parser(&plant);
    parser.AddModels(FindResourceOrThrow(model_file));
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
      std::filesystem::path model_path(model_file);
      if (model_path.filename() == "dual_iiwa14_polytope_collision.urdf") {
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
  parser.AddModels(FindResourceOrThrow(GetPlanarIiwaModelFile()));
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

// Tests that KUKA LBR iiwa14 models have consistent inertias.
// It takes iiwa14_no_collisions.sdf as the canonical model.
// It assumes all links are declared in the same order.
// It checks values directly on urdf files, generated from xacro.
GTEST_TEST(InertiasIiwa14, TestInertiaValues) {
  multibody::MultibodyPlant<double> canonical_plant(0.0);
  multibody::ModelInstanceIndex canonical_model_instance =
      LoadIiwa14CanonicalModel(&canonical_plant);
  canonical_plant.Finalize();

  const std::vector<multibody::BodyIndex> body_canonical_indices =
      canonical_plant.GetBodyIndices(canonical_model_instance);

  std::vector<std::string> model_files = GetCommonIiwa14ModelFiles();
  model_files.push_back(GetDualIiwaModelFile());
  model_files.push_back(GetPlanarIiwaModelFile());

  std::vector<std::string> link_names = {
      "iiwa_link_0", "iiwa_link_1", "iiwa_link_2", "iiwa_link_3",
      "iiwa_link_4", "iiwa_link_5", "iiwa_link_6", "iiwa_link_7"};

  for (auto& model_file : model_files) {
    SCOPED_TRACE(fmt::format("model file: {}", model_file));
    multibody::MultibodyPlant<double> plant(0.0);
    multibody::Parser parser(&plant);
    parser.AddModels(FindResourceOrThrow(model_file));
    plant.Finalize();
    std::filesystem::path model_path(model_file);
    if (model_path.filename() == "dual_iiwa14_polytope_collision.urdf") {
      for (size_t i = 0; i < link_names.size(); ++i) {
        SCOPED_TRACE(fmt::format("Link: {}", link_names[i]));
        const multibody::RigidBody<double>& canonical_body =
            canonical_plant.GetBodyByName(link_names[i]);
        const multibody::RigidBody<double>& left_robot_body =
            plant.GetBodyByName("left_" + link_names[i]);
        const multibody::RigidBody<double>& right_robot_body =
            plant.GetBodyByName("right_" + link_names[i]);
        CompareRotationalInertias(canonical_body, left_robot_body);
        CompareRotationalInertias(canonical_body, right_robot_body);
      }
    } else {
      for (size_t i = 0; i < link_names.size(); ++i) {
        SCOPED_TRACE(fmt::format("Link: {}", link_names[i]));
        const multibody::RigidBody<double>& canonical_body =
            canonical_plant.GetBodyByName(link_names[i]);
        const multibody::RigidBody<double>& robot_body =
            plant.GetBodyByName(link_names[i]);
        CompareRotationalInertias(canonical_body, robot_body);
      }
    }
  }
}

}  // namespace
}  // namespace manipulation
}  // namespace drake
