#include "drake/examples/kuka_iiwa_arm/dev/pick_and_place/pick_and_place_configuration_parsing.h"

#include <string>
#include <vector>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/examples/kuka_iiwa_arm/pick_and_place/pick_and_place_configuration.h"
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

const std::vector<Vector3<double>> kObjectPositions{{0.30, -0.9, 1.05}};

const std::vector<Vector3<double>> kObjectRpy{{0.0, 0.0, 1.571}};

const char kConfigurationFile[] =
    "drake/examples/kuka_iiwa_arm/dev/pick_and_place/configuration/"
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
  EXPECT_EQ(plant_configuration.static_friction_coef,
            expected_plant_configuration.static_friction_coef);
  EXPECT_EQ(plant_configuration.dynamic_friction_coef,
            expected_plant_configuration.dynamic_friction_coef);
  EXPECT_EQ(plant_configuration.v_stiction_tolerance,
            expected_plant_configuration.v_stiction_tolerance);
  EXPECT_EQ(plant_configuration.stiffness,
            expected_plant_configuration.stiffness);
  EXPECT_EQ(plant_configuration.dissipation,
            expected_plant_configuration.dissipation);
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
  EXPECT_EQ(configuration.model_path, expected_configuration.model_path);
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

    plant_configuration_.stiffness = 3e3;
    plant_configuration_.dissipation = 5;

    // Set planner parameters
    planner_configuration_.model_path = kIiwaPath;
    planner_configuration_.end_effector_name = kEndEffectorName;
    planner_configuration_.target_dimensions = kTargetDimensions;
    planner_configuration_.num_tables = 6;
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

TEST_F(ConfigurationParsingTests,
       ParsePlannerConfigurationRobotAndTargetIndex) {
  PlannerConfiguration parsed_planner_configuration =
      ParsePlannerConfigurationOrThrow(kConfigurationFile, "iiwa_link_ee",
                                       RobotBaseIndex(0), TargetIndex(0));
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
}  // namespace
}  // namespace pick_and_place
}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake
