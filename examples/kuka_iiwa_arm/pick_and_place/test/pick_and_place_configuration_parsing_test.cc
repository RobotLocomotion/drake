#include "drake/examples/kuka_iiwa_arm/pick_and_place/pick_and_place_configuration_parsing.h"

#include <string>
#include <vector>

#include <gmock/gmock.h>
#include <google/protobuf/text_format.h>
#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/examples/kuka_iiwa_arm/pick_and_place/pick_and_place_configuration.h"
#include "drake/examples/kuka_iiwa_arm/pick_and_place/pick_and_place_configuration.pb.h"
#include "drake/math/roll_pitch_yaw.h"

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {
namespace pick_and_place {
namespace {

using math::rpy2rotmat;
using pick_and_place::PlannerConfiguration;
using pick_and_place::SimulatedPlantConfiguration;
using pick_and_place::OptitrackConfiguration;
using pick_and_place::OptitrackInfo;
using pick_and_place::RobotBaseIndex;
using pick_and_place::TargetIndex;

const char kIiwaPath[] =
    "drake/manipulation/models/iiwa_description/urdf/"
    "iiwa14_polytope_collision.urdf";
const char kEndEffectorName[] = "iiwa_link_ee";
const char kYellowPostPath[] =
    "drake/examples/kuka_iiwa_arm/models/objects/yellow_post.urdf";
const char kExtraHeavyDutyTablePath[] =
    "drake/examples/kuka_iiwa_arm/models/table/"
    "extra_heavy_duty_table_surface_only_collision.sdf";
const char kCubePath[] =
    "drake/examples/kuka_iiwa_arm/models/objects/simple_cuboid.urdf";
const Vector3<double> kTargetDimensions{0.06, 0.06, 0.06};

const std::vector<int> kTableOptitrackIds{2, 3, 4, 5, 6, 7};

const std::vector<std::string> kTableModelPaths{
    kYellowPostPath,           // position A
    kYellowPostPath,           // position B
    kExtraHeavyDutyTablePath,  // position C
    kYellowPostPath,           // position D
    kYellowPostPath,           // position E
    kYellowPostPath,           // position F
};

const std::vector<Vector3<double>> kTablePositions{
    {0.00, 1.00, 0.0},     // position A
    {0.80, 0.36, 0.0},     // position B
    {0.86, -0.36, -0.07},  // position C
    {0.30, -0.9, 0.0},     // position D
    {-0.1, -1.0, 0.0},     // position E
    {-0.47, -0.8, 0.0},    // position F
};

const std::vector<double> kTableOptitrackFrameHeights{
    1.02,    // position A
    1.02,    // position B
    0.7645,  // position C
    1.02,    // position D
    1.02,    // position E
    1.02,    // position F
};

const std::vector<int> kObjectOptitrackIds{8};

const std::vector<std::string> kObjectModelPaths{kCubePath};

const std::vector<Vector3<double>> kObjectPositions{{0.80, 0.36, 1.05}};

const std::vector<Vector3<double>> kObjectRpy{{0.0, 0.0, 0.0}};

const char kConfigurationFile[] =
    "drake/examples/kuka_iiwa_arm/pick_and_place/configuration/"
    "yellow_posts.pick_and_place_configuration";

::testing::AssertionResult CompareIsometry3Vectors(
    const std::vector<Isometry3<double>>& pose_vector,
    const std::vector<Isometry3<double>>& expected_pose_vector) {
  if (pose_vector.size() != expected_pose_vector.size()) {
    return ::testing::AssertionFailure()
           << "Size mismatch: " << pose_vector.size() << " vs. "
           << expected_pose_vector.size();
  }
  for (int i = 0; i < static_cast<int>(pose_vector.size()); ++i) {
    ::testing::AssertionResult result{CompareMatrices(
        pose_vector[i].matrix(), expected_pose_vector[i].matrix())};
    if (!result) {
      return ::testing::AssertionFailure() << "Comparison failed at index " << i
                                           << ". " << result.message();
    }
  }
  return ::testing::AssertionSuccess();
}

::testing::AssertionResult CompareOptitrackInfoVectors(
    const std::vector<OptitrackInfo>& optitrack_info_vector,
    const std::vector<OptitrackInfo>& expected_optitrack_info_vector) {
  if (optitrack_info_vector.size() != expected_optitrack_info_vector.size()) {
    return ::testing::AssertionFailure()
           << "Size mismatch: " << optitrack_info_vector.size() << " vs. "
           << expected_optitrack_info_vector.size();
  }
  ::testing::AssertionResult result = ::testing::AssertionSuccess();
  for (int i = 0; i < static_cast<int>(optitrack_info_vector.size()); ++i) {
    result = CompareMatrices(optitrack_info_vector[i].X_MF.matrix(),
                             expected_optitrack_info_vector[i].X_MF.matrix());
    if (result &&
        optitrack_info_vector[i].id != expected_optitrack_info_vector[i].id) {
      result = ::testing::AssertionFailure()
               << "Optitrack ids do not match: " << optitrack_info_vector[i].id
               << " vs. " << expected_optitrack_info_vector[i].id;
    }
    if (!result) {
      return ::testing::AssertionFailure() << "Comparison failed at index " << i
                                           << ". " << result.message();
    }
  }
  return ::testing::AssertionSuccess();
}  // namespace

void ValidateSimulatedPlantConfiguration(
    const SimulatedPlantConfiguration& plant_configuration,
    const SimulatedPlantConfiguration& expected_plant_configuration) {
  EXPECT_TRUE(
      CompareIsometry3Vectors(plant_configuration.robot_poses,
                              expected_plant_configuration.robot_poses));
  EXPECT_TRUE(
      CompareIsometry3Vectors(plant_configuration.table_poses,
                              expected_plant_configuration.table_poses));
  EXPECT_TRUE(
      CompareIsometry3Vectors(plant_configuration.object_poses,
                              expected_plant_configuration.object_poses));
  EXPECT_EQ(plant_configuration.robot_models,
            expected_plant_configuration.robot_models);
  EXPECT_EQ(plant_configuration.table_models,
            expected_plant_configuration.table_models);
  EXPECT_EQ(plant_configuration.object_models,
            expected_plant_configuration.object_models);
  // Compliant contact model parameters
  const systems::CompliantContactModelParameters test_parameters =
      plant_configuration.contact_model_parameters;
  const systems::CompliantContactModelParameters expected_parameters =
      expected_plant_configuration.contact_model_parameters;
  EXPECT_EQ(test_parameters.characteristic_radius,
            expected_parameters.characteristic_radius);
  EXPECT_EQ(test_parameters.v_stiction_tolerance,
            expected_parameters.v_stiction_tolerance);

  // Default compliant material
  const systems::CompliantMaterial& test_material =
      plant_configuration.default_contact_material;
  const systems::CompliantMaterial& expected_material =
      expected_plant_configuration.default_contact_material;
  EXPECT_EQ(test_material.youngs_modulus(), expected_material.youngs_modulus());
  EXPECT_EQ(test_material.dissipation(), expected_material.dissipation());
  EXPECT_EQ(test_material.static_friction(),
            expected_material.static_friction());
  EXPECT_EQ(test_material.dynamic_friction(),
            expected_material.dynamic_friction());
}

void ValidateOptitrackConfiguration(
    const OptitrackConfiguration& configuration,
    const OptitrackConfiguration& expected_configuration) {
  EXPECT_TRUE(CompareOptitrackInfoVectors(
      configuration.robot_base_optitrack_info,
      expected_configuration.robot_base_optitrack_info));
  EXPECT_TRUE(CompareOptitrackInfoVectors(
      configuration.object_optitrack_info,
      expected_configuration.object_optitrack_info));
  EXPECT_TRUE(
      CompareOptitrackInfoVectors(configuration.table_optitrack_info,
                                  expected_configuration.table_optitrack_info));
}

void ValidatePlannerConfiguration(
    const PlannerConfiguration& configuration,
    const PlannerConfiguration& expected_configuration) {
  EXPECT_EQ(configuration.drake_relative_model_path,
            expected_configuration.drake_relative_model_path);
  EXPECT_EQ(configuration.end_effector_name,
            expected_configuration.end_effector_name);
  EXPECT_EQ(configuration.robot_index, expected_configuration.robot_index);
  EXPECT_EQ(configuration.target_dimensions,
            expected_configuration.target_dimensions);
  EXPECT_EQ(configuration.period_sec, expected_configuration.period_sec);
  EXPECT_EQ(configuration.num_tables, expected_configuration.num_tables);
}

class ConfigurationParsingTests : public ::testing::Test {
 protected:
  void SetUp() {
    // Set robot parameters
    optitrack_configuration_.robot_base_optitrack_info.emplace_back();
    optitrack_configuration_.robot_base_optitrack_info.back().id = 1;
    plant_configuration_.robot_poses.push_back(Isometry3<double>::Identity());
    plant_configuration_.robot_poses.back().translation().z() = 0.7645;
    plant_configuration_.robot_models.push_back(kIiwaPath);

    // Set table parameters
    const int num_tables = kTableModelPaths.size();
    for (int i = 0; i < num_tables; ++i) {
      optitrack_configuration_.table_optitrack_info.emplace_back();
      optitrack_configuration_.table_optitrack_info.back().id =
          kTableOptitrackIds[i];
      optitrack_configuration_.table_optitrack_info.back()
          .X_MF.translation()
          .z() += kTableOptitrackFrameHeights[i];
      plant_configuration_.table_models.push_back(kTableModelPaths[i]);
      plant_configuration_.table_poses.emplace_back(
          Isometry3<double>::Identity());
      plant_configuration_.table_poses.back().translation() =
          kTablePositions[i];
    }

    // Set object parameters
    const int num_objects = kObjectModelPaths.size();
    for (int i = 0; i < num_objects; ++i) {
      optitrack_configuration_.object_optitrack_info.emplace_back();
      optitrack_configuration_.object_optitrack_info.back().id =
          kObjectOptitrackIds[i];
      plant_configuration_.object_models.push_back(kObjectModelPaths[i]);
      plant_configuration_.object_poses.push_back(
          Isometry3<double>::Identity());
      plant_configuration_.object_poses.back().translation() =
          kObjectPositions[i];
      plant_configuration_.object_poses.back().rotate(
          AngleAxis<double>(kObjectRpy[i].z(), Vector3<double>::UnitZ()));
    }

    plant_configuration_.default_contact_material.set_youngs_modulus(3e7);
    plant_configuration_.default_contact_material.set_dissipation(5);

    // Set planner parameters
    planner_configuration_.drake_relative_model_path = kIiwaPath;
    planner_configuration_.end_effector_name = kEndEffectorName;
    planner_configuration_.target_dimensions = kTargetDimensions;
    planner_configuration_.num_tables = 6;
    planner_configuration_.grip_force = 42;
    planner_configuration_.grasp_frame_translational_offset = 0.191;
  }

  SimulatedPlantConfiguration plant_configuration_;
  pick_and_place::OptitrackConfiguration optitrack_configuration_;
  pick_and_place::PlannerConfiguration planner_configuration_;
};

TEST_F(ConfigurationParsingTests, ParsePlannerConfigurationTaskIndex) {
  PlannerConfiguration parsed_planner_configuration =
      ParsePlannerConfigurationOrThrow(kConfigurationFile, TaskIndex(0));
  ValidatePlannerConfiguration(parsed_planner_configuration,
                               planner_configuration_);
}

TEST_F(ConfigurationParsingTests, ParseSimulatedPlantConfiguration) {
  SimulatedPlantConfiguration parsed_plant_configuration =
      ParseSimulatedPlantConfigurationOrThrow(kConfigurationFile);
  ValidateSimulatedPlantConfiguration(parsed_plant_configuration,
                                      plant_configuration_);
}

TEST_F(ConfigurationParsingTests, ParseOptitrackConfiguration) {
  OptitrackConfiguration parsed_optitrack_configuration =
      ParseOptitrackConfigurationOrThrow(kConfigurationFile);
  ValidateOptitrackConfiguration(parsed_optitrack_configuration,
                                 optitrack_configuration_);
}

// Confirms that a configuration file with *no* compliant property specification
// is completely default-configured
GTEST_TEST(ConfigurationParsingCompliantParameterTests, AllDefaults) {
  SimulatedPlantConfiguration parsed_configuration =
      ParseSimulatedPlantConfigurationStringOrThrow("");

  systems::CompliantContactModelParameters default_parameters;
  const auto& parsed_parameters = parsed_configuration.contact_model_parameters;
  EXPECT_EQ(parsed_parameters.v_stiction_tolerance,
            default_parameters.v_stiction_tolerance);
  EXPECT_EQ(parsed_parameters.characteristic_radius,
            default_parameters.characteristic_radius);

  const auto& parsed_material = parsed_configuration.default_contact_material;
  EXPECT_TRUE(parsed_material.youngs_modulus_is_default());
  EXPECT_TRUE(parsed_material.dissipation_is_default());
  EXPECT_TRUE(parsed_material.friction_is_default());
}

// Confirms that bad configurations throw an appropriate exception.
GTEST_TEST(ConfigurationParsingCompliantParameterTests, BadValues) {
  // Values outside of their valid range
  EXPECT_THROW(ParseSimulatedPlantConfigurationStringOrThrow(
      "default_compliant_material { youngs_modulus: -1 }"),
               std::runtime_error);
  EXPECT_THROW(ParseSimulatedPlantConfigurationStringOrThrow(
      "default_compliant_material { dissipation: -1 }"),
               std::runtime_error);
  EXPECT_THROW(ParseSimulatedPlantConfigurationStringOrThrow(
      "default_compliant_material { static_friction_coefficient: -1 "
      "dynamic_friction_coefficient: -0.5 }"),
               std::runtime_error);
  EXPECT_THROW(ParseSimulatedPlantConfigurationStringOrThrow(
      "compliant_model_parameters { v_stiction_tolerance: -1 }"),
               std::runtime_error);
  EXPECT_THROW(ParseSimulatedPlantConfigurationStringOrThrow(
      "compliant_model_parameters { characteristic_radius: -1 }"),
               std::runtime_error);

  // Special friction logic -- only one defined and u_d > u_s.
  EXPECT_THROW(ParseSimulatedPlantConfigurationStringOrThrow(
      "default_compliant_material { static_friction_coefficient: 0.9 }"),
               std::runtime_error);
  EXPECT_THROW(ParseSimulatedPlantConfigurationStringOrThrow(
      "default_compliant_material { dynamic_friction_coefficient: 0.5 }"),
               std::runtime_error);
  EXPECT_THROW(ParseSimulatedPlantConfigurationStringOrThrow(
      "default_compliant_material { static_friction_coefficient: 0.5 "
      "dynamic_friction_coefficient: -0.9 }"),
               std::runtime_error);
}

// Confirms that all valid values get written to the configuration properly.
GTEST_TEST(ConfigurationParsingCompliantParameterTests, ConfiguredValues) {
  const char* configuration =
      R"_(
compliant_model_parameters {
  v_stiction_tolerance: 1e-3
  characteristic_radius: 1e-1
}

default_compliant_material {
  youngs_modulus: 1.5e7
  dissipation: 1.25
  static_friction_coefficient: 1.5
  dynamic_friction_coefficient: 1.0
}
)_";
  SimulatedPlantConfiguration parsed_configuration =
      ParseSimulatedPlantConfigurationStringOrThrow(configuration);

  const auto& parsed_parameters = parsed_configuration.contact_model_parameters;
  EXPECT_EQ(parsed_parameters.v_stiction_tolerance, 1e-3);
  EXPECT_EQ(parsed_parameters.characteristic_radius, 1e-1);

  const auto& parsed_material = parsed_configuration.default_contact_material;
  EXPECT_FALSE(parsed_material.youngs_modulus_is_default());
  EXPECT_EQ(parsed_material.youngs_modulus(), 1.5e7);
  EXPECT_FALSE(parsed_material.dissipation_is_default());
  EXPECT_EQ(parsed_material.dissipation(), 1.25);
  EXPECT_FALSE(parsed_material.friction_is_default());
  EXPECT_EQ(parsed_material.static_friction(), 1.5);
  EXPECT_EQ(parsed_material.dynamic_friction(), 1.0);
}

}  // namespace
}  // namespace pick_and_place
}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake
