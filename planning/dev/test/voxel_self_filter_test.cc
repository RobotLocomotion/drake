#include "drake/planning/dev/voxel_self_filter.h"

#include <functional>
#include <limits>
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

#include "drake/common/random.h"
#include "drake/common/text_logging.h"
#include "drake/geometry/scene_graph.h"
#include "drake/planning/dev/voxelized_environment_collision_checker.h"
#include "drake/planning/test/planning_test_helpers.h"

using common_robotics_utilities::math::Interpolate;

using drake::multibody::ModelInstanceIndex;

namespace drake {
namespace planning {
namespace test {
namespace {

GTEST_TEST(VoxelSelfFilterTest, Test) {
  // Assemble model directives.
  drake::multibody::parsing::ModelDirective add_robot_model;
  add_robot_model.add_model = drake::multibody::parsing::AddModel{
      "package://drake_models/iiwa_description/urdf/"
      "iiwa14_spheres_dense_elbow_collision.urdf",
      "iiwa"};
  drake::multibody::parsing::ModelDirective add_robot_weld;
  add_robot_weld.add_weld =
      drake::multibody::parsing::AddWeld{"world", "iiwa::base"};

  const drake::multibody::parsing::ModelDirectives directives{
      .directives = {add_robot_model, add_robot_weld}};

  auto robot_model = MakePlanningTestModel(directives);
  const std::vector<ModelInstanceIndex> robot_model_instances = {
      robot_model->plant().GetModelInstanceByName("iiwa")};

  // Parameters for the collision checker.
  const double edge_step_size = 0.05;
  const double env_padding = 0.0;
  const double self_padding = 0.0;

  // Make voxel collision checker
  std::unique_ptr<VoxelizedEnvironmentCollisionChecker> voxel_checker(
      new VoxelizedEnvironmentCollisionChecker(
          {.model = std::move(robot_model),
           .robot_model_instances = robot_model_instances,
           .configuration_distance_function =
               MakeWeightedIiwaConfigurationDistanceFunction(),
           .edge_step_size = edge_step_size,
           .env_collision_padding = env_padding,
           .self_collision_padding = self_padding}));

  // Make helper to sample random configurations
  const int num_positions = voxel_checker->plant().num_positions();
  const Eigen::VectorXd position_lower_limit =
      voxel_checker->plant().GetPositionLowerLimits();
  const Eigen::VectorXd position_upper_limit =
      voxel_checker->plant().GetPositionUpperLimits();

  RandomGenerator prng(42);

  const auto draw_random_q = [&]() -> Eigen::VectorXd {
    Eigen::VectorXd random_q(num_positions);
    for (int i = 0; i < num_positions; ++i) {
      const double random_draw =
          std::generate_canonical<double, std::numeric_limits<double>::digits>(
              prng);
      random_q(i) = Interpolate(position_lower_limit(i),
                                position_upper_limit(i), random_draw);
    }
    return random_q;
  };

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
  ASSERT_EQ(filled_environment.SetLocation(0.0, 0.0, 0.0, empty_cell),
            common_robotics_utilities::voxel_grid::AccessStatus::SUCCESS);

  // Check self-filter at a range of configurations.
  const double filter_padding = resolution * 0.5;
  const int32_t iterations = 100;
  int32_t iteration = 0;
  while (iteration < iterations) {
    const Eigen::VectorXd q = draw_random_q();
    const std::vector<Eigen::Isometry3d> X_WB_set =
        voxel_checker->ComputeBodyPoses(q);
    // Skip configs in self collision.
    if (voxel_checker->CheckSelfCollisionFree(X_WB_set)) {
      iteration += 1;
      voxel_checker->UpdateEnvironment("world", filled_environment);
      // The environment is filled, so it must be colliding.
      ASSERT_FALSE(voxel_checker->CheckConfigCollisionFree(q));
      auto working_environment = filled_environment;
      SelfFilter(*voxel_checker, q, filter_padding,
                 voxel_checker->plant().world_body().index(),
                 &working_environment, Parallelism::Max());
      voxel_checker->UpdateEnvironment("world", working_environment);
      // With self-filtering, the robot must no longer collide.
      ASSERT_TRUE(voxel_checker->CheckConfigCollisionFree(q));
    } else {
      drake::log()->info("Skipped config in self-collision");
    }
  }
}

}  // namespace
}  // namespace test
}  // namespace planning
}  // namespace drake
