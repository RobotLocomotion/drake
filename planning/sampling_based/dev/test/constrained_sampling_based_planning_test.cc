#include <memory>
#include <unordered_set>
#include <vector>

#include <Eigen/Geometry>
#include <common_robotics_utilities/print.hpp>
#include <gtest/gtest.h>

#include "drake/common/text_logging.h"
#include "drake/solvers/solve.h"
#include "common/anzu_model_directives.h"
#include "planning/interim_constrained_kinematic_planning_space.h"
#include "planning/make_planning_robot.h"
#include "planning/mbp_constraint_types.h"
#include "planning/sampling_based_planners.h"
#include "planning/test/planning_test_helpers.h"

namespace anzu {
namespace planning {
namespace {
using common_robotics_utilities::print::Print;
using drake::planning::CollisionCheckerContext;

GTEST_TEST(ConstrainedSamplingBasedPlanningTest, Test) {
  // Assemble model directives.
  drake::multibody::parsing::ModelDirective add_env_model;
  add_env_model.add_model = drake::multibody::parsing::AddModel{
      "package://anzu/models/test/collision_ground_plane.sdf",
      "ground_plane_box"};
  drake::multibody::parsing::ModelDirective add_env_weld;
  add_env_weld.add_weld = drake::multibody::parsing::AddWeld{
      "world", "ground_plane_box::ground_plane_box"};

  drake::multibody::parsing::ModelDirective add_robot_model;
  add_robot_model.add_model = drake::multibody::parsing::AddModel{
      "package://drake_models/iiwa_description/urdf/"
      "iiwa14_spheres_dense_elbow_collision.urdf", "iiwa"};
  drake::multibody::parsing::ModelDirective add_robot_weld;
  add_robot_weld.add_weld = drake::multibody::parsing::AddWeld{
      "world", "iiwa::base"};

  const std::vector<drake::multibody::parsing::ModelDirective> directives{
      add_env_model, add_env_weld, add_robot_model, add_robot_weld};

  planning::PlanningModelParams planning_model_params;
  planning_model_params.checker_type =
      planning::CollisionCheckerType::MbpEnvironmentChecker;
  planning_model_params.env_padding = 0.0;
  planning_model_params.self_padding = 0.0;
  planning_model_params.edge_step_size = 0.05;
  planning_model_params.prng_seed = 42;
  planning_model_params.robot_model_instance_names = {"iiwa"};
  planning_model_params.package_map = common::MakeDefaultAnzuPackageMap();
  planning_model_params.model_directives = directives;

  auto prototype_collision_checker =
      planning::MakeCollisionChecker(planning_model_params);
  const JointLimits joint_limits(prototype_collision_checker->plant());

  constexpr double propagation_step_size = 0.05;
  constexpr double minimum_propagation_progress = 0.01;
  constexpr double planning_tolerance = 1e-4;
  InterimConstrainedKinematicPlanningSpace planning_space(
      std::move(prototype_collision_checker), joint_limits,
      DummyConstraintsFactoryFunction, propagation_step_size,
      minimum_propagation_progress, planning_tolerance,
      planning_model_params.prng_seed);

  // Make constraints factory function
  const Eigen::Vector3d p_EE_constraint(0.0, 0.0, 0.0);
  const Eigen::Vector3d position_lower_bound(-1.0, -1.0, 0.095);
  const Eigen::Vector3d position_upper_bound(1.0, 1.0, 0.105);
  const auto& world_frame = planning_space.plant().world_frame();
  const auto& ee_frame = planning_space.plant().GetFrameByName("iiwa_link_ee");

  const ConstraintsFactoryFunction planning_constraints_factory_fn =
      [&](const std::shared_ptr<CollisionCheckerContext>& constraint_context) {
    std::vector<std::shared_ptr<drake::solvers::Constraint>> constraints;
    auto position_constraint = std::make_shared<StandalonePositionConstraint>(
        &planning_space.plant(), world_frame, position_lower_bound,
        position_upper_bound, ee_frame, p_EE_constraint, constraint_context);
    constraints.emplace_back(position_constraint);
    return constraints;
  };

  planning_space.SetConstraintsFactoryFunction(planning_constraints_factory_fn);

  // Make a separate set of constraints for checking
  const auto planning_constraints_context =
      planning_space.collision_checker().MakeStandaloneModelContext();
  const auto planning_constraints =
      planning_constraints_factory_fn(planning_constraints_context);

  // Sample two constrained configurations
  constexpr int max_tries = 100;

  const Eigen::VectorXd start = planning_space.SampleValidState(max_tries);
  const Eigen::VectorXd goal = planning_space.SampleValidState(max_tries);

  // Plan between them
  BiRRTPlanner<Eigen::VectorXd>::Parameters params;
  params.tree_sampling_bias = 0.5;
  params.p_switch_trees = 0.25;
  params.time_limit = 60.0;
  params.connection_tolerance = 1e-5;
  params.nearest_neighbor_parallelism = Parallelism::Max();

  const auto planning_result = BiRRTPlanner<Eigen::VectorXd>::Plan(
      start, goal, params, &planning_space);
  ASSERT_TRUE(planning_result.has_solution());
  ASSERT_GE(planning_result.path().size(), 2);
  drake::log()->debug(
      "Planned path with configs:\n{}",
      Print(planning_result.path(), false, "\n"));

  // Smooth
  PathProcessor<Eigen::VectorXd>::Parameters smoothing_params;
  smoothing_params.max_smoothing_shortcut_fraction = 1.0;
  smoothing_params.resampled_state_interval = 0.1;
  smoothing_params.prng_seed = 42;
  smoothing_params.max_smoothing_iterations = 200;
  smoothing_params.max_failed_smoothing_iterations = 200;
  smoothing_params.max_backtracking_steps = 0;
  smoothing_params.use_shortcut_smoothing = true;
  smoothing_params.resample_before_smoothing = false;
  smoothing_params.resample_shortcuts = false;
  smoothing_params.resample_after_smoothing = false;
  smoothing_params.check_for_marginal_shortcuts = false;
  smoothing_params.safety_check_path = false;

  const auto smoothed_path = PathProcessor<Eigen::VectorXd>::ProcessPath(
      planning_result.path(), smoothing_params, planning_space);
  drake::log()->debug(
      "Shortcut-smoothed path with configs:\n{}",
      Print(smoothed_path, false, "\n"));
  for (const auto& config : smoothed_path) {
    ASSERT_TRUE(planning::CheckConstraintsSatisfied(
        planning_constraints, config, planning_tolerance));
  }
}
}  // namespace
}  // namespace planning
}  // namespace anzu
