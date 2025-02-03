#include <cstdlib>
#include <functional>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Geometry>
#include <common_robotics_utilities/openmp_helpers.hpp>
#include <gtest/gtest.h>
#include <voxelized_geometry_tools/collision_map.hpp>

#include "drake/common/text_logging.h"
#include "drake/geometry/scene_graph.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/planning/collision_checker.h"
#include "planning/holonomic_kinematic_planning_space.h"
#include "planning/interim_constrained_kinematic_planning_space.h"
#include "planning/mbp_constraint_types.h"
#include "planning/sampling_based_planners.h"
#include "planning/test/planning_test_helpers.h"
#include "planning/voxelized_environment_collision_checker.h"

using drake::multibody::ModelInstanceIndex;
using drake::planning::CollisionCheckerContext;
using drake::planning::CollisionCheckerParams;

namespace anzu {
namespace planning {
namespace {
constexpr int64_t kPrngSeed = 42;
constexpr double kEnvPadding = 0.0625;
constexpr double kSelfPadding = 0.01;
constexpr int kMaxValidSampleTries = 100;

std::unique_ptr<VoxelizedEnvironmentCollisionChecker>
MakePlanningCollisionChecker() {
  // Assemble model directives.
  drake::multibody::parsing::ModelDirective add_robot_model;
  add_robot_model.add_model = drake::multibody::parsing::AddModel{
      "package://drake_models/iiwa_description/urdf/"
      "iiwa14_spheres_dense_collision.urdf", "iiwa"};
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
  // Make voxel collision checker
  return std::make_unique<VoxelizedEnvironmentCollisionChecker>(
      CollisionCheckerParams{
          .model = std::move(robot_model),
          .robot_model_instances = robot_model_instances,
          .configuration_distance_function = weighted_cspace_distance_fn,
          .edge_step_size = edge_step_size,
          .env_collision_padding = kEnvPadding,
          .self_collision_padding = kSelfPadding});
}

GTEST_TEST(ParallelBiRRTPlanningTest, UnconstrainedTest) {
  auto checker = MakePlanningCollisionChecker();

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
  // Frame name.
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

  // Make joint limits from plant.
  const JointLimits joint_limits(checker->plant());

  // Make the planning space.
  constexpr double kDefaultPropagationStepSize = 0.5;
  HolonomicKinematicPlanningSpace planning_space(
      std::move(checker), joint_limits, kDefaultPropagationStepSize, kPrngSeed);

  // Make planner & postprocessing parameters.
  ParallelBiRRTPlanner<Eigen::VectorXd>::Parameters parallel_birrt_parameters;
  parallel_birrt_parameters.tree_sampling_bias = 0.5;
  parallel_birrt_parameters.p_switch_trees = 0.25;
  parallel_birrt_parameters.time_limit = 60.0;
  parallel_birrt_parameters.connection_tolerance =
      std::numeric_limits<double>::epsilon();
  parallel_birrt_parameters.num_workers = 2;
  // This is an unusually low value to exercise tree swapping in test.
  parallel_birrt_parameters.initial_tree_capacity = 10;
  parallel_birrt_parameters.nearest_neighbor_parallelism =
      Parallelism::None();

  PathProcessor<Eigen::VectorXd>::Parameters postprocessing_params;
  postprocessing_params.max_smoothing_shortcut_fraction = 1.0;
  postprocessing_params.resampled_state_interval = 0.1;
  postprocessing_params.prng_seed = kPrngSeed;
  postprocessing_params.max_smoothing_iterations = 200;
  postprocessing_params.max_failed_smoothing_iterations = 200;
  postprocessing_params.max_backtracking_steps = 0;
  postprocessing_params.use_shortcut_smoothing = true;
  postprocessing_params.resample_before_smoothing = true;
  postprocessing_params.resample_shortcuts = false;
  postprocessing_params.resample_after_smoothing = false;
  postprocessing_params.check_for_marginal_shortcuts = false;
  postprocessing_params.safety_check_path = false;

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
    collision_checker.SetPaddingAllRobotEnvironmentPairs(kEnvPadding);
    collision_checker.SetPaddingAllRobotRobotPairs(kSelfPadding);
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

  check_planner_output(ParallelBiRRTPlanner<Eigen::VectorXd>::Plan(
      qs1, qg1, parallel_birrt_parameters, &planning_space));
  check_planner_output(ParallelBiRRTPlanner<Eigen::VectorXd>::Plan(
      qs1, qg2, parallel_birrt_parameters, &planning_space));
  check_planner_output(ParallelBiRRTPlanner<Eigen::VectorXd>::Plan(
      qs2, qg1, parallel_birrt_parameters, &planning_space));
  check_planner_output(ParallelBiRRTPlanner<Eigen::VectorXd>::Plan(
      qs2, qg2, parallel_birrt_parameters, &planning_space));

  // Multi-start/goal BiRRT
  check_planner_output(ParallelBiRRTPlanner<Eigen::VectorXd>::Plan(
      std::vector<Eigen::VectorXd>{qs1, qs2},
      std::vector<Eigen::VectorXd>{qg1, qg2},
      parallel_birrt_parameters, &planning_space));
}

GTEST_TEST(ParallelBiRRTPlanningTest, ConstrainedTest) {
  const auto checker = MakePlanningCollisionChecker();

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
  // Frame name.
  const std::string world_frame_name = "world";
  const voxelized_geometry_tools::CollisionCell empty_cell(0.0f);
  const voxelized_geometry_tools::CollisionCell filled_cell(1.0f);
  voxelized_geometry_tools::CollisionMap environment(
      X_WG, world_frame_name, grid_size, empty_cell);

  // Fill the bottom of the grid. Duplicating the walls of the unconstrained
  // test would interfere with the constrained motion.
  for (int64_t xidx = 0; xidx < environment.GetNumXCells(); xidx++) {
    for (int64_t yidx = 0; yidx < environment.GetNumYCells(); yidx++) {
      for (int64_t zidx = 0; zidx < environment.GetNumZCells(); zidx++) {
        if (zidx < 8) {
          environment.SetIndex(xidx, yidx, zidx, filled_cell);
        }
      }
    }
  }
  checker->UpdateEnvironment("world", environment);

  // Make joint limits from plant.
  const JointLimits joint_limits(checker->plant());

  const auto& ee_frame = checker->plant().GetFrameByName("iiwa_link_ee");
  const Eigen::Vector3d lower_bound(-1.0, -1.0, 0.095);
  const Eigen::Vector3d upper_bound(1.0, 1.0, 0.105);

  // Define the constraint factory function.
  const ConstraintsFactoryFunction constraints_factory_fn =
      [&](const std::shared_ptr<CollisionCheckerContext>&
              standalone_planning_context) {
        const auto planning_relative_position_constraint =
            std::make_shared<StandalonePositionConstraint>(
                &checker->plant(), checker->plant().world_frame(), lower_bound,
                upper_bound, ee_frame, Eigen::Vector3d(0.0, 0.0, 0.0),
                standalone_planning_context);
        return std::vector<std::shared_ptr<drake::solvers::Constraint>>{
            planning_relative_position_constraint};
      };

  // Make the planning space.
  constexpr double propagation_step_size = 0.05;
  constexpr double minimum_propagation_progress = 1e-5;
  constexpr double planning_tolerance = 1e-4;
  InterimConstrainedKinematicPlanningSpace planning_space(
      checker->Clone(), joint_limits, constraints_factory_fn,
      propagation_step_size, minimum_propagation_progress, planning_tolerance,
      kPrngSeed);

  // Make planner & postprocessing parameters.
  ParallelBiRRTPlanner<Eigen::VectorXd>::Parameters parallel_birrt_parameters;
  parallel_birrt_parameters.tree_sampling_bias = 0.5;
  parallel_birrt_parameters.p_switch_trees = 0.25;
  parallel_birrt_parameters.time_limit = 60.0;
  parallel_birrt_parameters.connection_tolerance =
      std::numeric_limits<double>::epsilon();
  parallel_birrt_parameters.num_workers = 2;
  // This is an unusually low value to exercise tree swapping in test.
  parallel_birrt_parameters.initial_tree_capacity = 10;
  parallel_birrt_parameters.nearest_neighbor_parallelism =
      Parallelism::None();

  PathProcessor<Eigen::VectorXd>::Parameters postprocessing_params;
  postprocessing_params.max_smoothing_shortcut_fraction = 1.0;
  postprocessing_params.resampled_state_interval = 0.1;
  postprocessing_params.prng_seed = kPrngSeed;
  postprocessing_params.max_smoothing_iterations = 200;
  postprocessing_params.max_failed_smoothing_iterations = 200;
  postprocessing_params.max_backtracking_steps = 0;
  postprocessing_params.use_shortcut_smoothing = true;
  postprocessing_params.resample_before_smoothing = false;
  postprocessing_params.resample_shortcuts = false;
  postprocessing_params.resample_after_smoothing = false;
  postprocessing_params.check_for_marginal_shortcuts = false;
  postprocessing_params.safety_check_path = false;

  // Sample start and target configurations.
  const Eigen::VectorXd qs1 =
      planning_space.SampleValidState(kMaxValidSampleTries);
  const Eigen::VectorXd qs2 =
      planning_space.SampleValidState(kMaxValidSampleTries);
  const Eigen::VectorXd qg1 =
      planning_space.SampleValidState(kMaxValidSampleTries);
  const Eigen::VectorXd qg2 =
      planning_space.SampleValidState(kMaxValidSampleTries);
  // Enforce that test configs are collision-free
  ASSERT_TRUE(planning_space.CheckStateValidity(qs1));
  ASSERT_TRUE(planning_space.CheckStateValidity(qs2));
  ASSERT_TRUE(planning_space.CheckStateValidity(qg1));
  ASSERT_TRUE(planning_space.CheckStateValidity(qg2));

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
    collision_checker.SetPaddingAllRobotEnvironmentPairs(kEnvPadding);
    collision_checker.SetPaddingAllRobotRobotPairs(kSelfPadding);
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
    drake::log()->info("Checking constrained path with {} states, length {}",
                        output.path().size(), output.path_length());
    check_path_for_collisions(output.path());
    const std::vector<Eigen::VectorXd> processed =
        PathProcessor<Eigen::VectorXd>::ProcessPath(
            output.path(), postprocessing_params, planning_space);
    drake::log()->info("Postprocessed from {} to {} states",
                        output.path().size(), processed.size());
    check_path_for_collisions(processed);
  };

  // Multi-start/goal BiRRT
  check_planner_output(ParallelBiRRTPlanner<Eigen::VectorXd>::Plan(
      std::vector<Eigen::VectorXd>{qs1, qs2},
      std::vector<Eigen::VectorXd>{qg1, qg2},
      parallel_birrt_parameters, &planning_space));
}
}  // namespace
}  // namespace planning
}  // namespace anzu
