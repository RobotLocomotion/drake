#include "drake/planning/sampling_based/dev/prm_planner.h"

#include <algorithm>
#include <utility>

#include <Eigen/Geometry>
#include <common_robotics_utilities/parallelism.hpp>
#include <common_robotics_utilities/simple_graph_search.hpp>
#include <common_robotics_utilities/simple_prm_planner.hpp>

#include "drake/common/drake_throw.h"
#include "drake/common/text_logging.h"
#include "drake/planning/sampling_based/dev/roadmap_internal.h"

namespace drake {
namespace planning {
// Don't add duplicate states to a roadmap.
const bool kAddDuplicateStates = false;
// Limit duplicate states added to the priority queue in graph search.
const bool kLimitPQueueDuplicates = true;
// Use roadmap overlays, rather than copying the roadmap.
const bool kUseRoadmapOverlay = true;

using common_robotics_utilities::parallelism::DegreeOfParallelism;
using common_robotics_utilities::simple_astar_search::AstarResult;
using common_robotics_utilities::simple_graph::NonOwningGraphOverlay;
using common_robotics_utilities::simple_graph_search::PerformLazyAstarSearch;
using common_robotics_utilities::simple_prm_planner::AddNodeToRoadmap;
using common_robotics_utilities::simple_prm_planner::BuildRoadMap;
using common_robotics_utilities::simple_prm_planner::ExtractSolution;
using common_robotics_utilities::simple_prm_planner::GrowRoadMap;
using common_robotics_utilities::simple_prm_planner::LazyQueryPath;
using common_robotics_utilities::simple_prm_planner::LazyQueryPathAndAddNodes;
using common_robotics_utilities::simple_prm_planner::LinearGraphKNNProvider;
using common_robotics_utilities::simple_prm_planner::NNDistanceDirection;
using common_robotics_utilities::simple_prm_planner::QueryPath;
using common_robotics_utilities::simple_prm_planner::QueryPathAndAddNodes;
using common_robotics_utilities::simple_prm_planner::UpdateRoadMapEdges;

template <typename StateType>
using LinearRoadmapKNN =
    LinearGraphKNNProvider<StateType, internal::RoadmapGraph<StateType>>;

// For functions that are only ever called by the main thread, we specify
// thread_number=0 to be consistent.
constexpr int kMainThreadNumber = 0;

namespace {

DegreeOfParallelism ToCRU(const Parallelism parallelism) {
  return DegreeOfParallelism(parallelism.num_threads());
}

// Helper to construct a PathPlanningResult from an AstarResult.
template <typename StateType>
PathPlanningResult<StateType> MakePathPlanningResult(
    const AstarResult<StateType>& planning_result) {
  if (planning_result.Path().size() > 0) {
    return PathPlanningResult<StateType>(planning_result.Path(),
                                         planning_result.PathCost());
  } else {
    return PathPlanningResult<StateType>(PathPlanningError::kCannotFindPath);
  }
}

}  // namespace

template <typename StateType>
Roadmap<StateType> PRMPlanner<StateType>::BuildRoadmap(
    const CreationParameters& parameters,
    PlanningSpace<StateType>* planning_space) {
  DRAKE_THROW_UNLESS(parameters.roadmap_size > 0);
  DRAKE_THROW_UNLESS(parameters.num_neighbors > 0);
  DRAKE_THROW_UNLESS(parameters.max_valid_sample_tries > 0);
  DRAKE_THROW_UNLESS(planning_space != nullptr);

  // Only sample valid states.
  const std::function<StateType(int32_t)> state_sampling_fn =
      [&](const int32_t thread_number) {
        return planning_space->SampleValidState(
            parameters.max_valid_sample_tries, thread_number);
      };
  // Since only valid states are sampled, state validity check is a no-op.
  const std::function<bool(int32_t, const StateType&)> state_validity_check_fn =
      [](const int32_t, const StateType&) {
        return true;
      };
  const std::function<double(const StateType&, const StateType&)>
      state_distance_fn = [&](const StateType& from, const StateType& to) {
        return planning_space->StateDistanceForwards(from, to);
      };
  // Since BuildRoadmap is already parallelized internally, further parallel
  // KNN with be counterproductive.
  const LinearRoadmapKNN<StateType> roadmap_knn(state_distance_fn,
                                                ToCRU(Parallelism::None()));
  const std::function<bool(int32_t, const StateType&, const StateType&)>
      edge_validity_check_fn = [&](const int32_t thread_num,
                                   const StateType& from, const StateType& to) {
        return planning_space->CheckEdgeValidity(from, to, thread_num);
      };

  const Parallelism connection_parallelism = std::min(
      parameters.connection_parallelism, planning_space->parallelism());

  auto built_roadmap_graph =
      BuildRoadMap<StateType, internal::RoadmapGraph<StateType>>(
          parameters.roadmap_size, state_sampling_fn, roadmap_knn,
          state_distance_fn, state_validity_check_fn, edge_validity_check_fn,
          parameters.num_neighbors,
          1 /* state_sampling_fn already incorporates multiple tries */,
          ToCRU(connection_parallelism), parameters.parallelize_sampling,
          planning_space->is_symmetric(),
          true /* allow duplicate states to be added for performance */);

  Roadmap<StateType> roadmap;
  auto& roadmap_graph = internal::GetMutableRoadmapInternalGraph(roadmap);
  roadmap_graph = std::move(built_roadmap_graph);
  return roadmap;
}

template <typename StateType>
Roadmap<StateType> PRMPlanner<StateType>::GrowRoadmap(
    const CreationParameters& parameters,
    PlanningSpace<StateType>* planning_space) {
  DRAKE_THROW_UNLESS(parameters.roadmap_size > 0);
  DRAKE_THROW_UNLESS(parameters.num_neighbors > 0);
  DRAKE_THROW_UNLESS(parameters.max_valid_sample_tries > 0);
  DRAKE_THROW_UNLESS(planning_space != nullptr);

  // Only sample valid states.
  const std::function<StateType(int32_t)> state_sampling_fn =
      [&](const int32_t thread_number) {
        return planning_space->SampleValidState(
            parameters.max_valid_sample_tries, thread_number);
      };
  // Since only valid states are sampled, state validity check is a no-op.
  const std::function<bool(int32_t, const StateType&)> state_validity_check_fn =
      [](const int32_t, const StateType&) {
        return true;
      };
  const std::function<double(const StateType&, const StateType&)>
      state_distance_fn = [&](const StateType& from, const StateType& to) {
        return planning_space->StateDistanceForwards(from, to);
      };
  const LinearRoadmapKNN<StateType> roadmap_knn(
      state_distance_fn, ToCRU(parameters.nearest_neighbor_parallelism));
  const std::function<bool(int32_t, const StateType&, const StateType&)>
      edge_validity_check_fn = [&](const int32_t thread_num,
                                   const StateType& from, const StateType& to) {
        return planning_space->CheckEdgeValidity(from, to, thread_num);
      };
  const std::function<bool(const int64_t)> termination_check_fn =
      [&](const int64_t current_roadmap_size) {
        return current_roadmap_size >=
               static_cast<int64_t>(parameters.roadmap_size);
      };

  const Parallelism connection_parallelism = std::min(
      parameters.connection_parallelism, planning_space->parallelism());

  Roadmap<StateType> roadmap(parameters.roadmap_size);

  auto& roadmap_graph = internal::GetMutableRoadmapInternalGraph(roadmap);

  const std::map<std::string, double> roadmap_growth_statistics =
      GrowRoadMap<StateType, internal::RoadmapGraph<StateType>>(
          roadmap_graph, state_sampling_fn, roadmap_knn, state_distance_fn,
          state_validity_check_fn, edge_validity_check_fn, termination_check_fn,
          parameters.num_neighbors, ToCRU(connection_parallelism),
          planning_space->is_symmetric(), kAddDuplicateStates);
  drake::log()->debug(
      "GrowRoadmap statistics:\n{}",
      common_robotics_utilities::print::Print(roadmap_growth_statistics));

  return roadmap;
}

template <typename StateType>
void PRMPlanner<StateType>::UpdateRoadmap(
    const PlanningSpace<StateType>& planning_space, Roadmap<StateType>* roadmap,
    const Parallelism parallelism) {
  DRAKE_THROW_UNLESS(roadmap != nullptr);

  const Parallelism connection_parallelism =
      std::min(parallelism, planning_space.parallelism());

  const std::function<double(const StateType&, const StateType&)>
      state_distance_fn = [&](const StateType& from, const StateType& to) {
        return planning_space.StateDistanceForwards(from, to);
      };
  const std::function<bool(int32_t, const StateType&, const StateType&)>
      edge_validity_check_fn = [&](const int32_t thread_num,
                                   const StateType& from, const StateType& to) {
        return planning_space.CheckEdgeValidity(from, to, thread_num);
      };

  auto& roadmap_graph = internal::GetMutableRoadmapInternalGraph(*roadmap);

  UpdateRoadMapEdges(roadmap_graph, edge_validity_check_fn, state_distance_fn,
                     ToCRU(connection_parallelism));
}

template <typename StateType>
PathPlanningResult<StateType> PRMPlanner<StateType>::Plan(
    const StateType& start, const StateType& goal,
    const QueryParameters& parameters,
    const PlanningSpace<StateType>& planning_space,
    const Roadmap<StateType>& roadmap) {
  return Plan(std::vector<StateType>{start}, std::vector<StateType>{goal},
              parameters, planning_space, roadmap);
}

template <typename StateType>
PathPlanningResult<StateType> PRMPlanner<StateType>::PlanAddingNodes(
    const StateType& start, const StateType& goal,
    const QueryParameters& parameters,
    const PlanningSpace<StateType>& planning_space,
    Roadmap<StateType>* roadmap) {
  return PlanAddingNodes(std::vector<StateType>{start},
                         std::vector<StateType>{goal}, parameters,
                         planning_space, roadmap);
}

template <typename StateType>
PathPlanningResult<StateType> PRMPlanner<StateType>::PlanLazy(
    const StateType& start, const StateType& goal,
    const QueryParameters& parameters,
    const PlanningSpace<StateType>& planning_space,
    const Roadmap<StateType>& roadmap) {
  return PlanLazy(std::vector<StateType>{start}, std::vector<StateType>{goal},
                  parameters, planning_space, roadmap);
}

template <typename StateType>
PathPlanningResult<StateType> PRMPlanner<StateType>::PlanLazyAddingNodes(
    const StateType& start, const StateType& goal,
    const QueryParameters& parameters,
    const PlanningSpace<StateType>& planning_space,
    Roadmap<StateType>* roadmap) {
  return PlanLazyAddingNodes(std::vector<StateType>{start},
                             std::vector<StateType>{goal}, parameters,
                             planning_space, roadmap);
}

template <typename StateType>
PathPlanningResult<StateType> PRMPlanner<StateType>::PlanEdgeValidity(
    const StateType& start, const StateType& goal,
    const QueryParameters& parameters,
    const PlanningSpace<StateType>& planning_space,
    const Roadmap<StateType>& roadmap,
    const std::vector<int32_t>& edge_validity_map,
    const StateOverrideFunction& state_override_fn) {
  return PlanEdgeValidity(
      std::vector<StateType>{start}, std::vector<StateType>{goal}, parameters,
      planning_space, roadmap, edge_validity_map, state_override_fn);
}

template <typename StateType>
PathPlanningResult<StateType> PRMPlanner<StateType>::Plan(
    const std::vector<StateType>& starts, const std::vector<StateType>& goals,
    const QueryParameters& parameters,
    const PlanningSpace<StateType>& planning_space,
    const Roadmap<StateType>& roadmap) {
  DRAKE_THROW_UNLESS(parameters.num_neighbors >= 0);

  const auto& [valid_starts, valid_goals, errors] =
      planning_space.ExtractValidStartsAndGoals(starts, goals,
                                                kMainThreadNumber);
  if (!errors.empty()) {
    return PathPlanningResult<StateType>(errors);
  }

  const Parallelism connection_parallelism =
      std::min(parameters.connection_parallelism, planning_space.parallelism());

  const std::function<double(const StateType&, const StateType&)>
      state_distance_fn = [&](const StateType& from, const StateType& to) {
        return planning_space.StateDistanceForwards(from, to);
      };
  const LinearRoadmapKNN<StateType> roadmap_knn(
      state_distance_fn, ToCRU(parameters.nearest_neighbor_parallelism));
  const std::function<bool(int32_t, const StateType&, const StateType&)>
      edge_validity_check_fn = [&](const int32_t thread_num,
                                   const StateType& from, const StateType& to) {
        return planning_space.CheckEdgeValidity(from, to, thread_num);
      };

  const auto& roadmap_graph = internal::GetRoadmapInternalGraph(roadmap);

  return MakePathPlanningResult<StateType>(QueryPath(
      valid_starts, valid_goals, roadmap_graph, roadmap_knn, state_distance_fn,
      edge_validity_check_fn, parameters.num_neighbors,
      ToCRU(connection_parallelism), planning_space.is_symmetric(),
      kAddDuplicateStates, kLimitPQueueDuplicates, kUseRoadmapOverlay));
}

template <typename StateType>
PathPlanningResult<StateType> PRMPlanner<StateType>::PlanAddingNodes(
    const std::vector<StateType>& starts, const std::vector<StateType>& goals,
    const QueryParameters& parameters,
    const PlanningSpace<StateType>& planning_space,
    Roadmap<StateType>* roadmap) {
  DRAKE_THROW_UNLESS(parameters.num_neighbors >= 0);
  DRAKE_THROW_UNLESS(roadmap != nullptr);

  const auto& [valid_starts, valid_goals, errors] =
      planning_space.ExtractValidStartsAndGoals(starts, goals,
                                                kMainThreadNumber);
  if (!errors.empty()) {
    return PathPlanningResult<StateType>(errors);
  }

  const Parallelism connection_parallelism =
      std::min(parameters.connection_parallelism, planning_space.parallelism());

  const std::function<double(const StateType&, const StateType&)>
      state_distance_fn = [&](const StateType& from, const StateType& to) {
        return planning_space.StateDistanceForwards(from, to);
      };
  const LinearRoadmapKNN<StateType> roadmap_knn(
      state_distance_fn, ToCRU(parameters.nearest_neighbor_parallelism));
  const std::function<bool(int32_t, const StateType&, const StateType&)>
      edge_validity_check_fn = [&](const int32_t thread_num,
                                   const StateType& from, const StateType& to) {
        return planning_space.CheckEdgeValidity(from, to, thread_num);
      };

  auto& roadmap_graph = internal::GetMutableRoadmapInternalGraph(*roadmap);

  return MakePathPlanningResult<StateType>(QueryPathAndAddNodes(
      valid_starts, valid_goals, roadmap_graph, roadmap_knn, state_distance_fn,
      edge_validity_check_fn, parameters.num_neighbors,
      ToCRU(connection_parallelism), planning_space.is_symmetric(),
      kAddDuplicateStates, kLimitPQueueDuplicates));
}

template <typename StateType>
PathPlanningResult<StateType> PRMPlanner<StateType>::PlanLazy(
    const std::vector<StateType>& starts, const std::vector<StateType>& goals,
    const QueryParameters& parameters,
    const PlanningSpace<StateType>& planning_space,
    const Roadmap<StateType>& roadmap) {
  DRAKE_THROW_UNLESS(parameters.num_neighbors >= 0);

  const auto& [valid_starts, valid_goals, errors] =
      planning_space.ExtractValidStartsAndGoals(starts, goals,
                                                kMainThreadNumber);
  if (!errors.empty()) {
    return PathPlanningResult<StateType>(errors);
  }

  const Parallelism connection_parallelism =
      std::min(parameters.connection_parallelism, planning_space.parallelism());

  const std::function<double(const StateType&, const StateType&)>
      state_distance_fn = [&](const StateType& from, const StateType& to) {
        return planning_space.StateDistanceForwards(from, to);
      };
  const LinearRoadmapKNN<StateType> roadmap_knn(
      state_distance_fn, ToCRU(parameters.nearest_neighbor_parallelism));
  const std::function<bool(int32_t, const StateType&, const StateType&)>
      edge_validity_check_fn = [&](const int32_t thread_num,
                                   const StateType& from, const StateType& to) {
        return planning_space.CheckEdgeValidity(from, to, thread_num);
      };

  const auto& roadmap_graph = internal::GetRoadmapInternalGraph(roadmap);

  return MakePathPlanningResult<StateType>(LazyQueryPath(
      valid_starts, valid_goals, roadmap_graph, roadmap_knn, state_distance_fn,
      edge_validity_check_fn, parameters.num_neighbors,
      ToCRU(connection_parallelism), planning_space.is_symmetric(),
      kAddDuplicateStates, kLimitPQueueDuplicates, kUseRoadmapOverlay));
}

template <typename StateType>
PathPlanningResult<StateType> PRMPlanner<StateType>::PlanLazyAddingNodes(
    const std::vector<StateType>& starts, const std::vector<StateType>& goals,
    const QueryParameters& parameters,
    const PlanningSpace<StateType>& planning_space,
    Roadmap<StateType>* roadmap) {
  DRAKE_THROW_UNLESS(parameters.num_neighbors >= 0);
  DRAKE_THROW_UNLESS(roadmap != nullptr);

  const auto& [valid_starts, valid_goals, errors] =
      planning_space.ExtractValidStartsAndGoals(starts, goals,
                                                kMainThreadNumber);
  if (!errors.empty()) {
    return PathPlanningResult<StateType>(errors);
  }

  const Parallelism connection_parallelism =
      std::min(parameters.connection_parallelism, planning_space.parallelism());

  const std::function<double(const StateType&, const StateType&)>
      state_distance_fn = [&](const StateType& from, const StateType& to) {
        return planning_space.StateDistanceForwards(from, to);
      };
  const LinearRoadmapKNN<StateType> roadmap_knn(
      state_distance_fn, ToCRU(parameters.nearest_neighbor_parallelism));
  const std::function<bool(int32_t, const StateType&, const StateType&)>
      edge_validity_check_fn = [&](const int32_t thread_num,
                                   const StateType& from, const StateType& to) {
        return planning_space.CheckEdgeValidity(from, to, thread_num);
      };

  auto& roadmap_graph = internal::GetMutableRoadmapInternalGraph(*roadmap);

  return MakePathPlanningResult<StateType>(LazyQueryPathAndAddNodes(
      valid_starts, valid_goals, roadmap_graph, roadmap_knn, state_distance_fn,
      edge_validity_check_fn, parameters.num_neighbors,
      ToCRU(connection_parallelism), planning_space.is_symmetric(),
      kAddDuplicateStates, kLimitPQueueDuplicates));
}

template <typename StateType>
PathPlanningResult<StateType> PRMPlanner<StateType>::PlanEdgeValidity(
    const std::vector<StateType>& starts, const std::vector<StateType>& goals,
    const QueryParameters& parameters,
    const PlanningSpace<StateType>& planning_space,
    const Roadmap<StateType>& roadmap,
    const std::vector<int32_t>& edge_validity_map,
    const StateOverrideFunction& state_override_fn) {
  DRAKE_THROW_UNLESS(parameters.num_neighbors >= 0);
  DRAKE_THROW_UNLESS(state_override_fn != nullptr);

  const auto& [valid_starts, valid_goals, errors] =
      planning_space.ExtractValidStartsAndGoals(starts, goals,
                                                kMainThreadNumber);
  if (!errors.empty()) {
    return PathPlanningResult<StateType>(errors);
  }

  const Parallelism connection_parallelism =
      std::min(parameters.connection_parallelism, planning_space.parallelism());

  const auto& roadmap_graph = internal::GetRoadmapInternalGraph(roadmap);

  using OverlaidRoadmap =
      NonOwningGraphOverlay<StateType, internal::RoadmapGraph<StateType>>;
  OverlaidRoadmap overlaid_roadmap(roadmap_graph);

  // Distance and edge validity functions for connecting start and goal states.
  const std::function<double(const StateType&, const StateType&)>
      state_distance_fn = [&](const StateType& from, const StateType& to) {
        const StateType override_from = state_override_fn(from);
        const StateType override_to = state_override_fn(to);
        return planning_space.StateDistanceForwards(override_from, override_to);
      };
  const LinearRoadmapKNN<StateType> roadmap_knn(
      state_distance_fn, ToCRU(parameters.nearest_neighbor_parallelism));
  const std::function<bool(int32_t, const StateType&, const StateType&)>
      edge_validity_check_fn = [&](const int32_t thread_num,
                                   const StateType& from, const StateType& to) {
        const StateType override_from = state_override_fn(from);
        const StateType override_to = state_override_fn(to);
        return planning_space.CheckEdgeValidity(override_from, override_to,
                                                thread_num);
      };

  // Edge validity check for roadmap edges.
  const std::function<bool(const OverlaidRoadmap&,
                           const typename OverlaidRoadmap::EdgeType&)>
      roadmap_edge_validity_check_fn =
          [&](const OverlaidRoadmap&,
              const typename OverlaidRoadmap::EdgeType& edge) {
            const uint64_t identifier = edge.GetScratchpad();
            if (identifier > 0) {
              // All edge identifiers are >= 1.
              const int32_t validity = edge_validity_map.at(identifier - 1);
              // Edges with validity = 1 are valid
              // Edges with validity = 2 are unknown
              // Edges with validity = 0 are invalid
              return (validity == 1);
            } else {
              // The only edges that don't have identifiers are the ones to the
              // start and goal nodes that have just been added. We know that
              // these edges are collision-free.
              return true;
            }
          };

  // Distance function for edges in the roadmap.
  const std::function<double(const OverlaidRoadmap&,
                             const typename OverlaidRoadmap::EdgeType&)>
      edge_distance_fn = [&](const OverlaidRoadmap&,
                             const typename OverlaidRoadmap::EdgeType& edge) {
        return edge.GetWeight();
      };

  // Heuristic function for nodes in the roadmap.
  const std::function<double(const OverlaidRoadmap&, int64_t, int64_t)>
      heuristic_fn = [&](const OverlaidRoadmap& overlaid_roadmap_graph,
                         const int64_t from_index, const int64_t to_index) {
        return planning_space.StateDistanceForwards(
            overlaid_roadmap_graph.GetNodeImmutable(from_index)
                .GetValueImmutable(),
            overlaid_roadmap_graph.GetNodeImmutable(to_index)
                .GetValueImmutable());
      };

  // Add start states to the roadmap.
  const int64_t pre_starts_size = overlaid_roadmap.Size();
  std::vector<int64_t> start_node_indices;
  for (const StateType& start : valid_starts) {
    const int64_t node_index =
        AddNodeToRoadmap(start, NNDistanceDirection::NEW_STATE_TO_ROADMAP,
                         overlaid_roadmap, roadmap_knn, state_distance_fn,
                         edge_validity_check_fn, parameters.num_neighbors,
                         pre_starts_size, ToCRU(connection_parallelism),
                         planning_space.is_symmetric(), kAddDuplicateStates);
    start_node_indices.emplace_back(node_index);
  }

  // Add goal states to the roadmap.
  const int64_t pre_goals_size = overlaid_roadmap.Size();
  std::vector<int64_t> goal_node_indices;
  for (const StateType& goal : valid_goals) {
    const int64_t node_index = AddNodeToRoadmap(
        goal, NNDistanceDirection::ROADMAP_TO_NEW_STATE, overlaid_roadmap,
        roadmap_knn, state_distance_fn, edge_validity_check_fn,
        parameters.num_neighbors, pre_goals_size, ToCRU(connection_parallelism),
        planning_space.is_symmetric(), kAddDuplicateStates);
    goal_node_indices.emplace_back(node_index);
  }

  // Call graph A* to find path.
  const AstarResult<int64_t> astar_result = PerformLazyAstarSearch(
      overlaid_roadmap, start_node_indices, goal_node_indices,
      roadmap_edge_validity_check_fn, edge_distance_fn, heuristic_fn,
      kLimitPQueueDuplicates);

  // Extract the solution path found by A*.
  const auto raw_result = ExtractSolution<StateType, std::vector<StateType>>(
      overlaid_roadmap, astar_result);

  if (raw_result.Path().size() > 0) {
    std::vector<StateType> override_solution_path;
    override_solution_path.reserve(raw_result.Path().size());
    for (const StateType& raw_path_state : raw_result.Path()) {
      override_solution_path.emplace_back(state_override_fn(raw_path_state));
    }
    return PathPlanningResult<StateType>(override_solution_path,
                                         raw_result.PathCost());
  } else {
    return PathPlanningResult<StateType>(PathPlanningError::kCannotFindPath);
  }
}

}  // namespace planning
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_PLANNING_STATE_TYPES(
    class ::drake::planning::PRMPlanner)
