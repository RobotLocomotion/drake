#include <cstdlib>
#include <functional>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Geometry>
#include <gtest/gtest.h>
#include <voxelized_geometry_tools/collision_map.hpp>

#include "drake/common/text_logging.h"
#include "drake/geometry/scene_graph.h"
#include "drake/planning/collision_checker.h"
#include "drake/planning/sampling_based/dev/holonomic_kinematic_planning_space.h"
#include "drake/planning/sampling_based/dev/sampling_based_planners.h"
#include "drake/planning/sampling_based/dev/test/planning_test_helpers.h"
#include "drake/planning/sampling_based/dev/voxelized_environment_collision_checker.h"

using drake::multibody::ModelInstanceIndex;

namespace drake {
namespace planning {
namespace {
GTEST_TEST(SamplingBasedPlanningTest, Test) {
  // Test planners.
  const int64_t prng_seed = 42;
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

  // Collision checker parameters.
  const auto weighted_cspace_distance_fn =
      MakeWeightedIiwaConfigurationDistanceFunction();
  const double edge_step_size = 0.05;
  const double env_padding = 0.0625;
  const double self_padding = 0.0;
  // Make voxel collision checker
  std::unique_ptr<VoxelizedEnvironmentCollisionChecker> checker(
      new VoxelizedEnvironmentCollisionChecker(
          {.model = std::move(robot_model),
           .robot_model_instances = robot_model_instances,
           .configuration_distance_function = weighted_cspace_distance_fn,
           .edge_step_size = edge_step_size,
           .env_collision_padding = env_padding,
           .self_collision_padding = self_padding}));

  // Make a basic environment model
  // Center 2x2x2m grid around the origin
  const Eigen::Isometry3d X_WG(Eigen::Translation3d(-1.0, -1.0, -1.0));
  // Grid 2m in each axis
  const double x_size = 2.0;
  const double y_size = 2.0;
  const double z_size = 2.0;
  // 1/8 meter resolution, so 16 cells/axis
  const double resolution = 0.125;
  const common_robotics_utilities::voxel_grid::GridSizes grid_size(
      resolution, x_size, y_size, z_size);
  // Frame name
  const std::string world_frame_name = "world";
  const voxelized_geometry_tools::CollisionCell empty_cell(0.0f);
  const voxelized_geometry_tools::CollisionCell filled_cell(1.0f);
  voxelized_geometry_tools::CollisionMap environment(
      X_WG, world_frame_name, grid_size, empty_cell);
  // Add vertical walls aligned to the X-axis in front and behind the robot.
  const std::vector<int64_t> x_indices =
      {0, 1, 2, 3, 4, 11, 12, 13, 14, 15};
  const std::vector<int64_t> y_indices = {7, 8};
  for (const int64_t xidx : x_indices) {
    for (const int64_t yidx : y_indices) {
      for (int64_t zidx = 0; zidx < environment.GetNumZCells(); zidx++) {
        environment.SetIndex(xidx, yidx, zidx, filled_cell);
      }
    }
  }
  checker->UpdateEnvironment("world", environment);

  // Get joint limits
  const JointLimits joint_limits(checker->plant());

  // Create a planning space
  constexpr double kPropagationStepSize = 0.5;
  HolonomicKinematicPlanningSpace planning_space(
      std::move(checker), joint_limits, kPropagationStepSize, prng_seed);

  // Assemble planner & postprocessing parameters
  constexpr int kRoadmapSize = 10000;
  constexpr int kNumNeighbors = 10;
  const auto kParallelism = Parallelism::Max();

  PRMPlanner<Eigen::VectorXd>::CreationParameters roadmap_parameters;
  roadmap_parameters.roadmap_size = kRoadmapSize;
  roadmap_parameters.num_neighbors = kNumNeighbors;
  roadmap_parameters.max_valid_sample_tries = 100;
  roadmap_parameters.nearest_neighbor_parallelism = kParallelism;
  roadmap_parameters.connection_parallelism = kParallelism;
  roadmap_parameters.parallelize_sampling = false;

  PRMPlanner<Eigen::VectorXd>::QueryParameters query_parameters;
  query_parameters.num_neighbors = kNumNeighbors;
  query_parameters.nearest_neighbor_parallelism = kParallelism;
  query_parameters.connection_parallelism = kParallelism;

  BiRRTPlanner<Eigen::VectorXd>::Parameters birrt_parameters;
  birrt_parameters.tree_sampling_bias = 0.5;
  birrt_parameters.p_switch_trees = 0.25;
  birrt_parameters.time_limit = 60.0;
  birrt_parameters.connection_tolerance =
      std::numeric_limits<double>::epsilon();
  birrt_parameters.nearest_neighbor_parallelism = kParallelism;

  PathProcessor<Eigen::VectorXd>::Parameters postprocessing_params;
  postprocessing_params.max_smoothing_shortcut_fraction = 1.0;
  postprocessing_params.resampled_state_interval = 0.1;
  postprocessing_params.prng_seed = prng_seed;
  postprocessing_params.max_smoothing_iterations = 1000;
  postprocessing_params.max_failed_smoothing_iterations = 1000;
  postprocessing_params.max_backtracking_steps = 1;
  postprocessing_params.use_shortcut_smoothing = true;
  postprocessing_params.resample_before_smoothing = true;
  postprocessing_params.resample_shortcuts = true;
  postprocessing_params.resample_after_smoothing = true;
  postprocessing_params.check_for_marginal_shortcuts = false;
  postprocessing_params.safety_check_path = false;

  // Build a roadmap
  drake::log()->info("Building roadmap...");
  const Roadmap<Eigen::VectorXd> roadmap =
      PRMPlanner<Eigen::VectorXd>::BuildRoadmap(
          roadmap_parameters, &planning_space);
  ASSERT_EQ(roadmap.Size(), kRoadmapSize);
  drake::log()->info("...roadmap built");

  // Test the planners
  // Make a helper to check paths
  auto check_path_for_collisions =
      [&] (const std::vector<Eigen::VectorXd>& path) {
    auto& collision_checker = planning_space.mutable_collision_checker();
    ASSERT_GE(path.size(), 2);
    // Since resampling in path postprocessing is not guaranteed to make
    // all arbitrary intermediate edge configurations collision free at the
    // original padding, we turn off padding for this check.
    collision_checker.SetPaddingAllRobotEnvironmentPairs(0.0);
    collision_checker.SetPaddingAllRobotRobotPairs(0.0);
    for (size_t idx = 0; idx < (path.size() - 1); idx++) {
      const Eigen::VectorXd& q1 = path.at(idx);
      const Eigen::VectorXd& q2 = path.at(idx + 1);
      ASSERT_TRUE(planning_space.CheckEdgeValidity(q1, q2));
    }
    // Restore padding
    collision_checker.SetPaddingAllRobotEnvironmentPairs(env_padding);
    collision_checker.SetPaddingAllRobotRobotPairs(self_padding);
    drake::log()->info(
        "Path with {} states checked collision-free", path.size());
  };

  // Make a helper to check planner success
  auto check_planner_output =
      [&] (const PathPlanningResult<Eigen::VectorXd>& output) {
    ASSERT_TRUE(output.has_solution());
    ASSERT_GE(output.path().size(), 2);
    ASSERT_FALSE(std::isinf(output.path_length()));
    ASSERT_FLOAT_EQ(
        output.path_length(), planning_space.CalcPathLength(output.path()));
    drake::log()->info("Checking path with {} states, length {}",
                        output.path().size(), output.path_length());
    check_path_for_collisions(output.path());
    const std::vector<Eigen::VectorXd> processed =
        PathProcessor<Eigen::VectorXd>::ProcessPath(
            output.path(), postprocessing_params, planning_space);
    drake::log()->info("Postprocessed from {} to {} states",
                        output.path().size(), processed.size());
    check_path_for_collisions(processed);
  };

  // Test configs
  Eigen::VectorXd qs1 = Eigen::VectorXd::Zero(7);
  qs1 << M_PI_4, M_PI_4, 0.0, M_PI_2, 0.0, 0.0, 0.0;
  Eigen::VectorXd qs2 = Eigen::VectorXd::Zero(7);
  qs2 << M_PI_4, M_PI_4, 0.0, -M_PI_2, 0.0, 0.0, 0.0;
  Eigen::VectorXd qg1 = Eigen::VectorXd::Zero(7);
  qg1 << -M_PI_4, M_PI_4, 0.0, M_PI_2, 0.0, 0.0, 0.0;
  Eigen::VectorXd qg2 = Eigen::VectorXd::Zero(7);
  qg2 << -M_PI_4, M_PI_4, 0.0, -M_PI_2, 0.0, 0.0, 0.0;
  // Enforce that test configs are collision-free
  ASSERT_TRUE(planning_space.CheckStateValidity(qs1));
  ASSERT_TRUE(planning_space.CheckStateValidity(qs2));
  ASSERT_TRUE(planning_space.CheckStateValidity(qg1));
  ASSERT_TRUE(planning_space.CheckStateValidity(qg2));

  // Basic PRM
  check_planner_output(PRMPlanner<Eigen::VectorXd>::Plan(
      qs1, qg1, query_parameters, planning_space, roadmap));
  check_planner_output(PRMPlanner<Eigen::VectorXd>::Plan(
      qs1, qg2, query_parameters, planning_space, roadmap));
  check_planner_output(PRMPlanner<Eigen::VectorXd>::Plan(
      qs2, qg1, query_parameters, planning_space, roadmap));
  check_planner_output(PRMPlanner<Eigen::VectorXd>::Plan(
      qs2, qg2, query_parameters, planning_space, roadmap));

  auto roadmap_copy = roadmap;
  check_planner_output(PRMPlanner<Eigen::VectorXd>::PlanAddingNodes(
      qs1, qg1, query_parameters, planning_space, &roadmap_copy));
  ASSERT_EQ(roadmap_copy.Size(), kRoadmapSize + 2);
  roadmap_copy = roadmap;
  check_planner_output(PRMPlanner<Eigen::VectorXd>::PlanAddingNodes(
      qs1, qg2, query_parameters, planning_space, &roadmap_copy));
  ASSERT_EQ(roadmap_copy.Size(), kRoadmapSize + 2);
  roadmap_copy = roadmap;
  check_planner_output(PRMPlanner<Eigen::VectorXd>::PlanAddingNodes(
      qs2, qg1, query_parameters, planning_space, &roadmap_copy));
  ASSERT_EQ(roadmap_copy.Size(), kRoadmapSize + 2);
  roadmap_copy = roadmap;
  check_planner_output(PRMPlanner<Eigen::VectorXd>::PlanAddingNodes(
      qs2, qg2, query_parameters, planning_space, &roadmap_copy));
  ASSERT_EQ(roadmap_copy.Size(), kRoadmapSize + 2);

  // Lazy-PRM
  check_planner_output(PRMPlanner<Eigen::VectorXd>::PlanLazy(
      qs1, qg1, query_parameters, planning_space, roadmap));
  check_planner_output(PRMPlanner<Eigen::VectorXd>::PlanLazy(
      qs1, qg2, query_parameters, planning_space, roadmap));
  check_planner_output(PRMPlanner<Eigen::VectorXd>::PlanLazy(
      qs2, qg1, query_parameters, planning_space, roadmap));
  check_planner_output(PRMPlanner<Eigen::VectorXd>::PlanLazy(
      qs2, qg2, query_parameters, planning_space, roadmap));

  roadmap_copy = roadmap;
  check_planner_output(PRMPlanner<Eigen::VectorXd>::PlanLazyAddingNodes(
      qs1, qg1, query_parameters, planning_space, &roadmap_copy));
  ASSERT_EQ(roadmap_copy.Size(), kRoadmapSize + 2);
  roadmap_copy = roadmap;
  check_planner_output(PRMPlanner<Eigen::VectorXd>::PlanLazyAddingNodes(
      qs1, qg2, query_parameters, planning_space, &roadmap_copy));
  ASSERT_EQ(roadmap_copy.Size(), kRoadmapSize + 2);
  roadmap_copy = roadmap;
  check_planner_output(PRMPlanner<Eigen::VectorXd>::PlanLazyAddingNodes(
      qs2, qg1, query_parameters, planning_space, &roadmap_copy));
  ASSERT_EQ(roadmap_copy.Size(), kRoadmapSize + 2);
  roadmap_copy = roadmap;
  check_planner_output(PRMPlanner<Eigen::VectorXd>::PlanLazyAddingNodes(
      qs2, qg2, query_parameters, planning_space, &roadmap_copy));
  ASSERT_EQ(roadmap_copy.Size(), kRoadmapSize + 2);

  // Multi-start/goal basic PRM
  check_planner_output(PRMPlanner<Eigen::VectorXd>::Plan(
      std::vector<Eigen::VectorXd>{qs1, qs2},
      std::vector<Eigen::VectorXd>{qg1, qg2},
      query_parameters, planning_space, roadmap));

  roadmap_copy = roadmap;
  check_planner_output(PRMPlanner<Eigen::VectorXd>::PlanAddingNodes(
      std::vector<Eigen::VectorXd>{qs1, qs2},
      std::vector<Eigen::VectorXd>{qg1, qg2},
      query_parameters, planning_space, &roadmap_copy));
  ASSERT_EQ(roadmap_copy.Size(), kRoadmapSize + 4);

  // Multi-start/goal Lazy-PRM
  check_planner_output(PRMPlanner<Eigen::VectorXd>::PlanLazy(
      std::vector<Eigen::VectorXd>{qs1, qs2},
      std::vector<Eigen::VectorXd>{qg1, qg2},
      query_parameters, planning_space, roadmap));

  roadmap_copy = roadmap;
  check_planner_output(PRMPlanner<Eigen::VectorXd>::PlanLazyAddingNodes(
      std::vector<Eigen::VectorXd>{qs1, qs2},
      std::vector<Eigen::VectorXd>{qg1, qg2},
      query_parameters, planning_space, &roadmap_copy));
  ASSERT_EQ(roadmap_copy.Size(), kRoadmapSize + 4);

  // BiRRT
  check_planner_output(BiRRTPlanner<Eigen::VectorXd>::Plan(
      qs1, qg1, birrt_parameters, &planning_space));
  check_planner_output(BiRRTPlanner<Eigen::VectorXd>::Plan(
      qs1, qg2, birrt_parameters, &planning_space));
  check_planner_output(BiRRTPlanner<Eigen::VectorXd>::Plan(
      qs2, qg1, birrt_parameters, &planning_space));
  check_planner_output(BiRRTPlanner<Eigen::VectorXd>::Plan(
      qs2, qg2, birrt_parameters, &planning_space));

  // Multi-start/goal BiRRT
  check_planner_output(BiRRTPlanner<Eigen::VectorXd>::Plan(
      std::vector<Eigen::VectorXd>{qs1, qs2},
      std::vector<Eigen::VectorXd>{qg1, qg2},
      birrt_parameters, &planning_space));
}
}  // namespace
}  // namespace planning
}  // namespace drake
