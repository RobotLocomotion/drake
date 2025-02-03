#pragma once

#include <cstdint>
#include <optional>
#include <vector>

#include <spdlog/spdlog.h>

#include "planning/default_state_types.h"
#include "planning/parallelism.h"
#include "planning/path_planning_result.h"
#include "planning/planning_space.h"

namespace anzu {
namespace planning {
/// Parallel Bi-directional RRT planner. Thread safety of planner trees is
/// provided by a copy-on-grow approach.
template<typename StateType>
class ParallelBiRRTPlanner {
 public:
  /// Parameters to the Parallel BiRRT planner.
  // TODO(calderpg) Provide/document good defaults.
  struct Parameters {
    /// Probability that sampled states should come from the target tree.
    double tree_sampling_bias{0.0};
    /// Probability that the active tree should switch after each iteration.
    double p_switch_trees{0.0};
    /// Time limit for planning. A time limit, a tree growth limit, or both,
    /// must be provided.
    std::optional<double> time_limit;
    /// Maximum tree growth (the sum of both start and goal trees) limit for
    /// planning. A time limit, a tree growth limit, or both, must be provided.
    /// If provided, value must be greater than 2 (there will always be at least
    /// one state in both start and goal trees).
    std::optional<int> tree_growth_limit;
    /// Tolerance for connect checks between start and goal tree.
    double connection_tolerance{0.0};
    /// Number of parallel workers to use. Planning methods throw if the
    /// specified number of workers exceeds the parallelism of the provided
    /// planning space.
    int num_workers{0};
    /// Initial capacity of planner trees. Larger values require more memory,
    /// but reduce the frequency of copy-on-grow tree swaps.
    int initial_tree_capacity{0};
    /// How should nearest neighbor checks be parallelized? To be performed in
    /// parallel, both this parameter must enable it, and the planning space
    /// must support parallel operations. To avoid resource starvation with this
    /// enabled, the number of threads used by nearest neighbors ("nn_threads")
    /// must be set so that the total num_threads >= num_workers * nn_threads.
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
      const StateType& start,
      const StateType& goal,
      const Parameters& parameters,
      PlanningSpace<StateType>* planning_space);

  /// Plan a path from the provided start states to goal states.
  /// @param starts Starting states.
  /// @param goals Ending states.
  /// @param parameters Parameters to planner.
  /// @param planning_space Planning space to use. @pre not null.
  /// @return First path found from *a* start to *a* goal, if found in time
  /// limit.
  static PathPlanningResult<StateType> Plan(
      const std::vector<StateType>& starts,
      const std::vector<StateType>& goals,
      const Parameters& parameters,
      PlanningSpace<StateType>* planning_space);

  // Delete all constructors of this static-only class.
  ParallelBiRRTPlanner(const ParallelBiRRTPlanner&) = delete;
};
}  // namespace planning
}  // namespace anzu

ANZU_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_PLANNING_STATE_TYPES(
    class ::anzu::planning::ParallelBiRRTPlanner)
