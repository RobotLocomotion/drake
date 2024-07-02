#include "drake/planning/dev/voxelized_environment_collision_checker.h"

#include <common_robotics_utilities/voxel_grid.hpp>
#include <gtest/gtest.h>
#include <voxelized_geometry_tools/collision_map.hpp>

#include "drake/planning/dev/sphere_robot_model_collision_checker.h"
#include "drake/planning/dev/test/sphere_robot_model_collision_checker_abstract_test_suite.h"
#include "drake/planning/dev/voxelized_environment_builder.h"
#include "drake/planning/test/planning_test_helpers.h"
#include "drake/planning/test_utilities/collision_checker_abstract_test_suite.h"

namespace drake {
namespace planning {
namespace test {
namespace {

CollisionCheckerTestParams MakeVoxelizedEnvironmentCollisionCheckerParams() {
  CollisionCheckerTestParams result;
  const CollisionCheckerConstructionParams p;
  auto model = MakePlanningTestModel(MakeCollisionCheckerTestScene());
  const auto robot_instance = model->plant().GetModelInstanceByName("iiwa");

  // Make Voxel-based collision checker, keeping a derived-type pointer for
  // further configuration steps.
  auto voxel_checker = new VoxelizedEnvironmentCollisionChecker(
      {.model = std::move(model),
       .robot_model_instances = {robot_instance},
       .configuration_distance_function =
           MakeWeightedIiwaConfigurationDistanceFunction(),
       .edge_step_size = p.edge_step_size,
       .env_collision_padding = p.env_padding,
       .self_collision_padding = p.self_padding});
  result.checker.reset(voxel_checker);

  // Make a voxelized copy of the environment.
  const std::string world_frame_name = "world";
  // Center grid at world origin.
  const Eigen::Isometry3d X_WG(Eigen::Translation3d(-1.0, -1.0, -1.0));
  // Grid is 2m in each axis.
  const Eigen::Vector3d grid_size(2.0, 2.0, 2.0);
  // Use 1/8 meter resolution, so 16 cells/axis.
  const double grid_resolution = 0.125;
  // Voxelize.
  const voxelized_geometry_tools::CollisionMap voxelized_environment =
      BuildCollisionMap(
          voxel_checker->plant(), voxel_checker->plant_context(),
          static_cast<SphereRobotModelCollisionChecker*>(voxel_checker)
              ->RobotGeometries(),
          world_frame_name, X_WG, grid_size, grid_resolution);
  for (int64_t xidx = 0; xidx < voxelized_environment.GetNumXCells(); xidx++) {
    for (int64_t yidx = 0; yidx < voxelized_environment.GetNumYCells();
         yidx++) {
      for (int64_t zidx = 0; zidx < voxelized_environment.GetNumZCells();
           zidx++) {
        const auto occupancy =
            voxelized_environment.GetIndexImmutable(xidx, yidx, zidx)
                .Value()
                .Occupancy();
        if (zidx >= 8) {
          DRAKE_ASSERT(occupancy == 0.0);
        } else {
          DRAKE_ASSERT(occupancy == 1.0);
        }
      }
    }
  }
  // Load the voxelized environment.
  voxel_checker->UpdateEnvironment("world", voxelized_environment);
  result.supports_added_world_obstacles = false;
  return result;
}

}  // namespace

INSTANTIATE_TEST_SUITE_P(
    VoxelizedEnvironmentCollisionCheckerTest, CollisionCheckerAbstractTestSuite,
    testing::Values(MakeVoxelizedEnvironmentCollisionCheckerParams()));

INSTANTIATE_TEST_SUITE_P(
    VoxelizedEnvironmentCollisionCheckerTest2,
    SphereRobotModelCollisionCheckerAbstractTestSuite,
    testing::Values(MakeVoxelizedEnvironmentCollisionCheckerParams().checker));

}  // namespace test
}  // namespace planning
}  // namespace drake
