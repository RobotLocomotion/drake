#include "planning/voxel_self_filter.h"

#include <functional>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Geometry>
#include <common_robotics_utilities/math.hpp>
#include <common_robotics_utilities/print.hpp>
#include <common_robotics_utilities/voxel_grid.hpp>
#include <gtest/gtest.h>
#include <voxelized_geometry_tools/collision_map.hpp>

#include "drake/common/text_logging.h"
#include "drake/geometry/scene_graph.h"
#include "planning/holonomic_kinematic_planning_space.h"
#include "planning/test/planning_test_helpers.h"
#include "planning/voxelized_environment_collision_checker.h"

using drake::multibody::ModelInstanceIndex;

namespace anzu {
namespace planning {
namespace {
GTEST_TEST(VoxelSelfFilterTest, Test) {
  // Assemble model directives.
  drake::multibody::parsing::ModelDirective add_robot_model;
  add_robot_model.add_model = drake::multibody::parsing::AddModel{
      "package://drake_models/iiwa_description/urdf/"
      "iiwa14_spheres_dense_elbow_collision.urdf", "iiwa"};
  drake::multibody::parsing::ModelDirective add_robot_weld;
  add_robot_weld.add_weld = drake::multibody::parsing::AddWeld{
      "world", "iiwa::base"};

  const drake::multibody::parsing::ModelDirectives directives{.directives = {
      add_robot_model, add_robot_weld}};

  auto robot_model = MakePlanningTestModel(directives);
  const std::vector<ModelInstanceIndex> robot_model_instances = {
      robot_model->plant().GetModelInstanceByName("iiwa")};

  // Parameters for the collision checker.
  const double edge_step_size = 0.05;
  const double env_padding = 0.0;
  const double self_padding = 0.0;

  // Make voxel collision checker
  std::unique_ptr<VoxelizedEnvironmentCollisionChecker> raw_checker(
      new VoxelizedEnvironmentCollisionChecker(
          {.model = std::move(robot_model),
           .robot_model_instances = robot_model_instances,
           .configuration_distance_function =
               MakeWeightedIiwaConfigurationDistanceFunction(),
           .edge_step_size = edge_step_size,
           .env_collision_padding = env_padding,
           .self_collision_padding = self_padding}));

  // Make planning space
  const JointLimits joint_limits(raw_checker->plant());
  HolonomicKinematicPlanningSpace planning_space(
      std::move(raw_checker), joint_limits, 0.05, 42);
  auto& voxel_checker = static_cast<VoxelizedEnvironmentCollisionChecker&>(
      planning_space.mutable_collision_checker());

  // Make a basic environment model
  // Grid centered around origin
  const Eigen::Isometry3d X_WG(Eigen::Translation3d(-1.0, -1.0, -1.0));
  // Grid 1m in each axis
  const double x_size = 2.0;
  const double y_size = 2.0;
  const double z_size = 2.0;
  // 1/32 meter resolution, so 64 cells/axis
  const double resolution = 0.03125;
  const common_robotics_utilities::voxel_grid::GridSizes grid_size(
      resolution, x_size, y_size, z_size);
  // Frame name
  const std::string world_frame_name = "world";
  const voxelized_geometry_tools::CollisionCell empty_cell(0.0f);
  const voxelized_geometry_tools::CollisionCell filled_cell(1.0f);
  voxelized_geometry_tools::CollisionMap filled_environment(
      X_WG, world_frame_name, grid_size, filled_cell);
  // We set a single cell empty to ensure a valid SDF is computed.
  ASSERT_EQ(
      filled_environment.SetLocation(0.0, 0.0, 0.0, empty_cell),
      common_robotics_utilities::voxel_grid::AccessStatus::SUCCESS);

  // Check self-filter at a range of configurations.
  const double filter_padding = resolution * 0.5;
  const int32_t iterations = 1000;
  int32_t iteration = 0;
  while (iteration < iterations) {
    const Eigen::VectorXd q = planning_space.SampleState();
    const std::vector<Eigen::Isometry3d> X_WB_set =
        voxel_checker.ComputeBodyPoses(q);
    // Skip configs in self collision.
    if (voxel_checker.CheckSelfCollisionFree(X_WB_set)) {
      iteration += 1;
      voxel_checker.UpdateEnvironment("world", filled_environment);
      // The environment is filled, so it must be colliding.
      ASSERT_FALSE(voxel_checker.CheckConfigCollisionFree(q));
      auto working_environment = filled_environment;
      SelfFilter(voxel_checker, q, filter_padding,
                 voxel_checker.plant().world_body().index(),
                 &working_environment, Parallelism::Max());
      voxel_checker.UpdateEnvironment("world", working_environment);
      // With self-filtering, the robot must no longer collide.
      ASSERT_TRUE(voxel_checker.CheckConfigCollisionFree(q));
    } else {
      drake::log()->info("Skipped config in self-collision");
    }
  }
}
}  // namespace
}  // namespace planning
}  // namespace anzu
