#pragma once

#include <cstdint>
#include <functional>
#include <optional>
#include <vector>

#include <spdlog/spdlog.h>

#include "drake/common/parallelism.h"
#include "drake/planning/sampling_based/dev/default_state_types.h"
#include "drake/planning/sampling_based/dev/goal_checker.h"
#include "drake/planning/sampling_based/dev/goal_sampler.h"
#include "drake/planning/sampling_based/dev/path_planning_result.h"
#include "drake/planning/sampling_based/dev/planning_space.h"

namespace drake {
namespace planning {
/// Single-directional RRT planner.
template <typename StateType>
class RRTPlanner {
 public:
  /// Parameters to the RRT planner.
  // TODO(calderpg) Provide/document good defaults.
  struct Parameters {
    /// Probability that the sampled state comes from the goal state(s)/goal
    /// sampling function. Ignored when using an implicit goal check function.
    double goal_sampling_bias{0.0};
    /// Probability that a sampled goal is a newly sampled goal, not one
    /// previously sampled. Only used with goal sampling.
    double p_goal_sample_is_new{0.0};
    /// Time limit for planning. A time limit, a tree growth limit, or both,
    /// must be provided.
    std::optional<double> time_limit;
    /// Maximum tree growth limit for planning. A time limit, a tree growth
    /// limit, or both, must be provided. If provided, value must be greater
    /// than 1 (there will always be at least one state in the tree).
    std::optional<int> tree_growth_limit;
    /// Tolerance for checks against goal states.
    double goal_tolerance{0.0};
    /// How should nearest neighbor checks be parallelized? To be performed in
    /// parallel, both this parameter must enable it, and the planning space
    /// must support parallel operations.
    Parallelism nearest_neighbor_parallelism{Parallelism::None()};
    /// Log level used for the planner's own log messages.
    spdlog::level::level_enum planner_log_level{spdlog::level::debug};
  };

  /// Plan a path from the provided start state to goal state.
  /// @param start Starting state.
  /// @param goal Ending state.
  /// @param parameters Parameters to planner.
  /// @param planning_space Planning space to use. @pre not null.
  /// @return First path found from start to goal, if found in time limit.
  static PathPlanningResult<StateType> Plan(
      const StateType& start, const StateType& goal,
      const Parameters& parameters, PlanningSpace<StateType>* planning_space);

  /// Plan a path from the provided start states to goal states.
  /// @param starts Starting states.
  /// @param goals Ending states.
  /// @param parameters Parameters to planner.
  /// @param planning_space Planning space to use. @pre not null.
  /// @return First path found from *a* start to *a* goal, if found in time
  /// limit.
  static PathPlanningResult<StateType> Plan(
      const std::vector<StateType>& starts, const std::vector<StateType>& goals,
      const Parameters& parameters, PlanningSpace<StateType>* planning_space);

  /// Plan a path from the provided start state to sampled goal state(s).
  /// @param start Starting state.
  /// @param goal_sampler Goal sampler.
  /// @param parameters Parameters to planner.
  /// @param planning_space Planning space to use. @pre not null.
  /// @return First path found from start to *a* goal, if found in time limit.
  static PathPlanningResult<StateType> PlanGoalSampling(
      const StateType& start, const GoalSampler<StateType>& goal_sampler,
      const Parameters& parameters, PlanningSpace<StateType>* planning_space);

  /// Plan a path from the provided start states to sampled goal state(s).
  /// @param starts Starting states.
  /// @param goal_sampler Goal sampler.
  /// @param parameters Parameters to planner.
  /// @param planning_space Planning space to use. @pre not null.
  /// @return First path found from *a* start to *a* goal, if found in time
  /// limit.
  static PathPlanningResult<StateType> PlanGoalSampling(
      const std::vector<StateType>& starts,
      const GoalSampler<StateType>& goal_sampler, const Parameters& parameters,
      PlanningSpace<StateType>* planning_space);

  /// Plan a path from the provided start state to the goal implicitly defined
  /// by the goal checker.
  /// @param start Starting state.
  /// @param goal_checker Goal checker.
  /// @param parameters Parameters to planner.
  /// @param planning_space Planning space to use. @pre not null.
  /// @return First path found from start to *a* state meeting the goal check,
  /// if found in time limit.
  static PathPlanningResult<StateType> PlanGoalCheck(
      const StateType& start, const GoalChecker<StateType>& goal_checker,
      const Parameters& parameters, PlanningSpace<StateType>* planning_space);

  /// Plan a path from the provided start states to the goal implicitly defined
  /// by the goal checker.
  /// @param starts Starting states.
  /// @param goal_checker Goal checker.
  /// @param parameters Parameters to planner.
  /// @param planning_space Planning space to use. @pre not null.
  /// @return First path found from *a* start to *a* state meeting the goal
  /// check, if found in time limit.
  static PathPlanningResult<StateType> PlanGoalCheck(
      const std::vector<StateType>& starts,
      const GoalChecker<StateType>& goal_checker, const Parameters& parameters,
      PlanningSpace<StateType>* planning_space);

  // Delete all constructors of this static-only class.
  RRTPlanner(const RRTPlanner&) = delete;
};
}  // namespace planning
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_PLANNING_STATE_TYPES(
    class ::drake::planning::RRTPlanner)
