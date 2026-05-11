#include "drake/planning/dev/voxel_self_filter.h"

#include <functional>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Geometry>
#include <common_robotics_utilities/math.hpp>
#include <gtest/gtest.h>

#include "drake/common/random.h"
#include "drake/common/text_logging.h"
#include "drake/geometry/scene_graph.h"
#include "drake/planning/dev/voxel_grid_internal.h"
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
  const math::RigidTransformd X_WG(Eigen::Vector3d(-1.0, -1.0, -1.0));
  // Grid +/-1m in each axis
  const Eigen::Vector3d grid_dimensions(2.0, 2.0, 2.0);
  // 1/32 meter resolution, so 64 cells/axis
  constexpr double grid_resolution = 0.03125;
  // Frame name
  const std::string world_frame_name = "world";

  constexpr float empty_occupancy = 0.0f;
  constexpr float filled_occupancy = 1.0f;

  VoxelOccupancyMap filled_environment(world_frame_name, X_WG, grid_dimensions,
                                       grid_resolution, filled_occupancy);

  // We set a single cell empty to ensure a valid SDF is computed.
  auto& internal_occupancy_map =
      internal::GetMutableInternalOccupancyMap(filled_environment);

  auto& zero_voxel =
      internal_occupancy_map.GetLocationMutable(0.0, 0.0, 0.0).Value();
  zero_voxel.SetOccupancy(empty_occupancy);

  // Check self-filter at a range of configurations.
  const double filter_padding = grid_resolution * 0.5;
  const int32_t iterations = 10;
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
