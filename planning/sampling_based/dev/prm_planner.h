#pragma once

#include <cstdint>
#include <map>
#include <string>
#include <vector>

#include "drake/common/parallelism.h"
#include "drake/planning/sampling_based/dev/default_state_types.h"
#include "drake/planning/sampling_based/dev/path_planning_result.h"
#include "drake/planning/sampling_based/dev/planning_space.h"
#include "drake/planning/sampling_based/dev/roadmap.h"

namespace drake {
namespace planning {
/// PRM planner.
template <typename StateType>
class PRMPlanner {
 public:
  /// Parameters for roadmap creation.
  // TODO(calderpg) Provide/document good defaults.
  struct CreationParameters {
    /// Size of the roadmap. @pre > 0.
    int roadmap_size{0};
    /// Number of neighbors to consider when creating the roadmap. @pre > 0.
    int num_neighbors{0};
    /// Maximum number of tries to sample a valid state. @pre > 0.
    int max_valid_sample_tries{0};
    /// How should nearest neighbor checks be parallelized? To be performed in
    /// parallel, both this parameter must enable it, and the planning space
    /// must support parallel operations.
    Parallelism nearest_neighbor_parallelism{Parallelism::None()};
    /// How should connection operations be parallelized? To be performed in
    /// parallel, both this parameter must enable it, and the planning space
    /// must support parallel operations. This does not enable parallel sampling
    /// in BuildRoadmap.
    Parallelism connection_parallelism{Parallelism::None()};
    /// Should sampling in BuildRoadmap also be parallelized if parallelization
    /// is enabled? Note that enabling parallel sampling means the sampled
    /// configurations will *not* be deterministic, although it is likely that
    /// with parallel sampling enabled the sampling sequence will be
    /// reproducible for a fixed number of OpenMP threads and OpenMP
    /// implementation, as sampling is performed in a by-default statically
    /// scheduled parallel-for loop.
    bool parallelize_sampling{false};
  };

  /// Parameters for roadmap planning queries.
  // TODO(calderpg) Provide/document good defaults.
  struct QueryParameters {
    /// Number of neighbors to consider when connecting the start and goal
    /// states to the roadmap. @pre > 0.
    int num_neighbors{0};
    /// How should nearest neighbor checks be parallelized? To be performed in
    /// parallel, both this parameter must enable it, and the planning space
    /// must support parallel operations.
    Parallelism nearest_neighbor_parallelism{Parallelism::None()};
    /// How should connection operations be parallelized? To be performed in
    /// parallel, both this parameter must enable it, and the planning space
    /// must support parallel operations.
    Parallelism connection_parallelism{Parallelism::None()};
  };

  using StateOverrideFunction = std::function<StateType(const StateType&)>;

  /// Create a roadmap using the "build roadmap" strategy that first samples all
  /// roadmap states, then connects them together. This approach is much faster
  /// than the "grow roadmap" strategy, but *must only* be used in spaces that
  /// do not sample duplicate states. If duplicate states are sampled, all the
  /// duplicate states will be added to the roadmap.
  /// @param parameters Parameters for roadmap creation.
  /// @param planning_space Planning space to use. @pre non null.
  /// @return Roadmap.
  static Roadmap<StateType> BuildRoadmap(
      const CreationParameters& parameters,
      PlanningSpace<StateType>* planning_space);

  /// Create a roadmap using the "grow roadmap" strategy that iteratively adds
  /// states to the roadmap. This is slower than the "build roadmap" strategy,
  /// but is more generally applicable.
  /// @param parameters Parameters for roadmap creation.
  /// @param planning_space Planning space to use. @pre non null.
  /// @return Roadmap.
  static Roadmap<StateType> GrowRoadmap(
      const CreationParameters& parameters,
      PlanningSpace<StateType>* planning_space);

  /// Update the validity and edge weights of edges in the provided roadmap.
  /// Invalid edges are set to infinite weight.
  /// @param planning_space Planning space to use.
  /// @param roadmap Roadmap to update. @pre non null.
  /// @param parallelism How should update be parallelized? To be performed in
  /// parallel both this parameter must be true, and the planning space must
  /// support parallel operations.
  static void UpdateRoadmap(const PlanningSpace<StateType>& planning_space,
                            Roadmap<StateType>* roadmap,
                            Parallelism parallelism);

  /// Plans a path through the provided roadmap from the provided start state to
  /// the provided goal state.
  /// @param start Starting state.
  /// @param goal Ending state.
  /// @param parameters Parameters to planner.
  /// @param planning_space Planning space to use.
  /// @param roadmap Roadmap to use.
  /// @return Shortest path between start and goal, if one exists.
  static PathPlanningResult<StateType> Plan(
      const StateType& start, const StateType& goal,
      const QueryParameters& parameters,
      const PlanningSpace<StateType>& planning_space,
      const Roadmap<StateType>& roadmap);

  /// Plans a path through the provided roadmap from the provided start states
  /// to the provided goal states.
  /// @param starts Starting states.
  /// @param goals Ending states.
  /// @param parameters Parameters to planner.
  /// @param planning_space Planning space to use.
  /// @param roadmap Roadmap to use.
  /// @return Shortest path between *a* start and *a* goal, if one exists.
  static PathPlanningResult<StateType> Plan(
      const std::vector<StateType>& starts, const std::vector<StateType>& goals,
      const QueryParameters& parameters,
      const PlanningSpace<StateType>& planning_space,
      const Roadmap<StateType>& roadmap);

  /// Plans a path through the provided roadmap from the provided start state to
  /// the provided goal state. Start and goal states are added to the roadmap.
  /// @param start Starting state.
  /// @param goal Ending state.
  /// @param parameters Parameters to planner.
  /// @param planning_space Planning space to use.
  /// @param roadmap Roadmap to use. @pre non null.
  /// @return Shortest path between start and goal, if one exists.
  static PathPlanningResult<StateType> PlanAddingNodes(
      const StateType& start, const StateType& goal,
      const QueryParameters& parameters,
      const PlanningSpace<StateType>& planning_space,
      Roadmap<StateType>* roadmap);

  /// Plans a path through the provided roadmap from the provided start states
  /// to the provided goal states. Start and goal states are added to the
  /// roadmap.
  /// @param starts Starting states.
  /// @param goals Ending states.
  /// @param parameters Parameters to planner.
  /// @param planning_space Planning space to use.
  /// @param roadmap Roadmap to use. @pre non null.
  /// @return Shortest path between *a* start and *a* goal, if one exists.
  static PathPlanningResult<StateType> PlanAddingNodes(
      const std::vector<StateType>& starts, const std::vector<StateType>& goals,
      const QueryParameters& parameters,
      const PlanningSpace<StateType>& planning_space,
      Roadmap<StateType>* roadmap);

  /// Plans a path through the provided roadmap from the provided start state to
  /// the provided goal state. Lazy planning means that edge feasiblity is
  /// queried as-needed by the roadmap search, and the edges of the provided
  /// roadmap are not assumed to be feasible.
  /// @param start Starting state.
  /// @param goal Ending state.
  /// @param parameters Parameters to planner.
  /// @param planning_space Planning space to use.
  /// @param roadmap Roadmap to use.
  /// @return Shortest path between start and goal, if one exists.
  static PathPlanningResult<StateType> PlanLazy(
      const StateType& start, const StateType& goal,
      const QueryParameters& parameters,
      const PlanningSpace<StateType>& planning_space,
      const Roadmap<StateType>& roadmap);

  /// Plans a path through the provided roadmap from the provided start states
  /// to the provided goal states. Lazy planning means that edge feasiblity is
  /// queried as-needed by the roadmap search, and the edges of the provided
  /// roadmap are not assumed to be feasible.
  /// @param starts Starting states.
  /// @param goals Ending states.
  /// @param parameters Parameters to planner.
  /// @param planning_space Planning space to use.
  /// @param roadmap Roadmap to use.
  /// @return Shortest path between *a* start and *a* goal, if one exists.
  static PathPlanningResult<StateType> PlanLazy(
      const std::vector<StateType>& starts, const std::vector<StateType>& goals,
      const QueryParameters& parameters,
      const PlanningSpace<StateType>& planning_space,
      const Roadmap<StateType>& roadmap);

  /// Plans a path through the provided roadmap from the provided start state to
  /// the provided goal state. Start and goal states are added to the roadmap.
  /// Lazy planning means that edge feasiblity is queried as-needed by the
  /// roadmap search, and the edges of the provided roadmap are not assumed to
  /// be feasible.
  /// @param start Starting state.
  /// @param goal Ending state.
  /// @param parameters Parameters to planner.
  /// @param planning_space Planning space to use.
  /// @param roadmap Roadmap to use. @pre non null.
  /// @return Shortest path between start and goal, if one exists.
  static PathPlanningResult<StateType> PlanLazyAddingNodes(
      const StateType& start, const StateType& goal,
      const QueryParameters& parameters,
      const PlanningSpace<StateType>& planning_space,
      Roadmap<StateType>* roadmap);

  /// Plans a path through the provided roadmap from the provided start states
  /// to the provided goal states. Start and goal states are added to the
  /// roadmap. Lazy planning means that edge feasiblity is queried as-needed by
  /// the graph search, and the edges of the provided roadmap are not assumed to
  /// be feasible.
  /// @param starts Starting states.
  /// @param goals Ending states.
  /// @param parameters Parameters to planner.
  /// @param planning_space Planning space to use.
  /// @param roadmap Roadmap to use. @pre non null.
  /// @return Shortest path between *a* start and *a* goal, if one exists.
  static PathPlanningResult<StateType> PlanLazyAddingNodes(
      const std::vector<StateType>& starts, const std::vector<StateType>& goals,
      const QueryParameters& parameters,
      const PlanningSpace<StateType>& planning_space,
      Roadmap<StateType>* roadmap);

  /// (Advanced)
  /// Plans a path through the provided roadmap from the provided start state to
  /// the provided goal state, using the provided edge validity map to identify
  /// which edges in the roadmap are valid. Note that no "add node" version may
  /// be provided, as the edge validity mechanism does not support modifications
  /// to the roadmap.
  /// @param start Starting state.
  /// @param goal Ending state.
  /// @param parameters Parameters to planner.
  /// @param planning_space Planning space to use.
  /// @param roadmap Roadmap to use.
  /// @param edge_validity_map Vector of edge feasibility (feasible = 1,
  /// colliding = 0, unknown = 2) for each distinct edge in the roadmap.
  /// Note int32_t is used as edge_validity_map generally is processed by GPU.
  /// @param state_override_fn Function to override states stored in roadmap.
  /// By default, the override function does nothing.
  /// @return Shortest path between start and goal, if one exists.
  static PathPlanningResult<StateType> PlanEdgeValidity(
      const StateType& start, const StateType& goal,
      const QueryParameters& parameters,
      const PlanningSpace<StateType>& planning_space,
      const Roadmap<StateType>& roadmap,
      const std::vector<int32_t>& edge_validity_map,
      const StateOverrideFunction& state_override_fn =
          [](const StateType& state) {
            return state;
          });

  /// (Advanced)
  /// Plans a path through the provided roadmap from the provided start states
  /// to the provided goal states, using the provided edge validity map to
  /// identify which edges in the roadmap are valid. Note that no "add node"
  /// version may be provided, as the edge validity mechanism does not support
  /// modifications to the roadmap.
  /// @param starts Starting states.
  /// @param goals Ending states.
  /// @param parameters Parameters to planner.
  /// @param planning_space Planning space to use.
  /// @param roadmap Roadmap to use.
  /// @param edge_validity_map Vector of edge feasibility (feasible = 1,
  /// colliding = 0, unknown = 2) for each distinct edge in the roadmap.
  /// Note int32_t is used as edge_validity_map generally is processed by GPU.
  /// @param state_override_fn Function to override states stored in roadmap.
  /// By default, the override function does nothing.
  /// @return Shortest path between *a* start and *a* goal, if one exists.
  static PathPlanningResult<StateType> PlanEdgeValidity(
      const std::vector<StateType>& starts, const std::vector<StateType>& goals,
      const QueryParameters& parameters,
      const PlanningSpace<StateType>& planning_space,
      const Roadmap<StateType>& roadmap,
      const std::vector<int32_t>& edge_validity_map,
      const StateOverrideFunction& state_override_fn =
          [](const StateType& state) {
            return state;
          });

  // Delete all constructors of this static-only class.
  PRMPlanner(const PRMPlanner&) = delete;
};
}  // namespace planning
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_PLANNING_STATE_TYPES(
    class ::drake::planning::PRMPlanner)
