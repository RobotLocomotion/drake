#include "planning/birrt_planner.h"

#include <map>
#include <string>

#include <common_robotics_utilities/print.hpp>
#include <common_robotics_utilities/simple_knearest_neighbors.hpp>
#include <common_robotics_utilities/simple_rrt_planner.hpp>

#include "drake/common/drake_throw.h"
#include "drake/common/text_logging.h"
#include "planning/rrt_internal.h"

namespace anzu {
namespace planning {
using common_robotics_utilities::simple_knearest_neighbors::
    GetKNearestNeighbors;
using common_robotics_utilities::simple_rrt_planner::BiRRTActiveTreeType;
using common_robotics_utilities::simple_rrt_planner::
    BiRRTNearestNeighborFunction;
using common_robotics_utilities::simple_rrt_planner::BiRRTPropagationFunction;
using common_robotics_utilities::simple_rrt_planner::
    BiRRTStatesConnectedFunction;
using common_robotics_utilities::simple_rrt_planner::
    BiRRTSelectSampleTypeFunction;
using common_robotics_utilities::simple_rrt_planner::BiRRTTreeSamplingFunction;
using common_robotics_utilities::simple_rrt_planner::SamplingFunction;
using common_robotics_utilities::simple_rrt_planner::
    BiRRTSelectActiveTreeFunction;
using common_robotics_utilities::simple_rrt_planner::
    MakeUniformRandomBiRRTSelectSampleTypeFunction;
using common_robotics_utilities::simple_rrt_planner::
    MakeUniformRandomBiRRTTreeSamplingFunction;
using common_robotics_utilities::simple_rrt_planner::
    MakeUniformRandomBiRRTSelectActiveTreeFunction;
using common_robotics_utilities::simple_rrt_planner::
    BiRRTTerminationCheckFunction;
using common_robotics_utilities::simple_rrt_planner::ForwardPropagation;
using common_robotics_utilities::simple_rrt_planner::BiRRTPlanSinglePath;
using common_robotics_utilities::simple_rrt_planner::SimpleRRTPlannerTree;
using common_robotics_utilities::utility::UniformUnitRealFunction;

// Functions that take thread numbers are only ever called by the BiRRT planner
// in the main thread, to be consistent we specify thread_number=0 everywhere.
constexpr int kMainThreadNumber = 0;

namespace {
template <typename StateType>
using TreeNodeDistanceFunction = std::function<double(
    const typename SimpleRRTPlannerTree<StateType>::NodeType&,
    const StateType&)>;

template <typename StateType>
int64_t GetRRTNearestNeighbor(
    const SimpleRRTPlannerTree<StateType>& tree, const StateType& sample,
    const TreeNodeDistanceFunction<StateType>& distance_fn,
    Parallelism parallelism) {
  const auto neighbors = GetKNearestNeighbors(
      tree.GetNodesImmutable(), sample, distance_fn, 1, ToCRU(parallelism));
  if (neighbors.size() > 0) {
    const auto& nearest_neighbor = neighbors.at(0);
    return nearest_neighbor.Index();
  } else {
    throw std::runtime_error("NN check produced no neighbors");
  }
}
}  // namespace

template<typename StateType>
PathPlanningResult<StateType> BiRRTPlanner<StateType>::Plan(
    const StateType& start,
    const StateType& goal,
    const Parameters& parameters,
    PlanningSpace<StateType>* const planning_space) {
  return Plan(std::vector<StateType>{start}, std::vector<StateType>{goal},
              parameters, planning_space);
}

template<typename StateType>
PathPlanningResult<StateType> BiRRTPlanner<StateType>::Plan(
    const std::vector<StateType>& starts,
    const std::vector<StateType>& goals,
    const Parameters& parameters,
    PlanningSpace<StateType>* const planning_space) {
  DRAKE_THROW_UNLESS(parameters.tree_sampling_bias > 0.0);
  DRAKE_THROW_UNLESS(parameters.tree_sampling_bias < 1.0);
  DRAKE_THROW_UNLESS(parameters.p_switch_trees > 0.0);
  DRAKE_THROW_UNLESS(parameters.p_switch_trees <= 1.0);
  DRAKE_THROW_UNLESS(parameters.connection_tolerance >= 0.0);
  DRAKE_THROW_UNLESS(planning_space != nullptr);

  internal::ValidateBiRRTTerminationConditions(parameters, "BiRRTPlanner");

  const auto& [valid_starts, valid_goals, errors] =
      planning_space->ExtractValidStartsAndGoals(
          starts, goals, kMainThreadNumber);
  if (!errors.empty()) {
    return PathPlanningResult<StateType>(errors);
  }

  // Build helper functions. For operations performed on/from the start tree, we
  // use the "forwards" methods provided by the planning space; for operations
  // on/from the goal tree, we use the "backwards" methods instead.

  // Sampling function.
  const SamplingFunction<StateType> sampling_fn = [&]() {
    return planning_space->SampleState(kMainThreadNumber);
  };

  // Nearest-neighbor function.
  const BiRRTNearestNeighborFunction<StateType> nearest_neighbor_fn = [&](
      const SimpleRRTPlannerTree<StateType>& tree,
      const StateType& sample,
      const BiRRTActiveTreeType active_tree_type) {
    switch (active_tree_type) {
      case BiRRTActiveTreeType::START_TREE:
        return GetRRTNearestNeighbor<StateType>(
            tree, sample,
            [&](const typename SimpleRRTPlannerTree<StateType>::NodeType& from,
                const StateType& to) {
              return planning_space->NearestNeighborDistanceForwards(
                  from.GetValueImmutable(), to);
            },
            parameters.nearest_neighbor_parallelism);
      case BiRRTActiveTreeType::GOAL_TREE:
        return GetRRTNearestNeighbor<StateType>(
            tree, sample,
            [&](const typename SimpleRRTPlannerTree<StateType>::NodeType& from,
                const StateType& to) {
              return planning_space->NearestNeighborDistanceBackwards(
                  from.GetValueImmutable(), to);
            },
            parameters.nearest_neighbor_parallelism);
    }
    DRAKE_UNREACHABLE();
  };

  // Statistics for edge propagation function.
  std::map<std::string, double> propagation_statistics;

  // Edge propagation function.
  const BiRRTPropagationFunction<StateType> propagation_fn =
      [&](const StateType& nearest, const StateType& sampled,
          const BiRRTActiveTreeType active_tree_type) {
    std::vector<StateType> propagated_states;

    switch (active_tree_type) {
      case BiRRTActiveTreeType::START_TREE:
        propagated_states = planning_space->PropagateForwards(
            nearest, sampled, &propagation_statistics, kMainThreadNumber);
        break;
      case BiRRTActiveTreeType::GOAL_TREE:
        propagated_states = planning_space->PropagateBackwards(
            nearest, sampled, &propagation_statistics, kMainThreadNumber);
        break;
    }

    return internal::MakeForwardPropagation(propagated_states);
  };

  // State-state connection check function.
  const BiRRTStatesConnectedFunction<StateType> states_connected_fn = [&](
      const StateType& from, const StateType& to,
      const BiRRTActiveTreeType active_tree_type) {
    double distance = 0.0;
    switch (active_tree_type) {
      case BiRRTActiveTreeType::START_TREE:
        distance = planning_space->StateDistanceForwards(from, to);
        break;
      case BiRRTActiveTreeType::GOAL_TREE:
        distance = planning_space->StateDistanceBackwards(from, to);
        break;
    }
    return distance <= parameters.connection_tolerance;
  };

  // Assemble starts & goals.
  SimpleRRTPlannerTree<StateType> start_tree(valid_starts.size());
  for (const StateType& start : valid_starts) {
    start_tree.AddNode(start);
  }
  SimpleRRTPlannerTree<StateType> goal_tree(valid_goals.size());
  for (const StateType& goal : valid_goals) {
    goal_tree.AddNode(goal);
  }

  const UniformUnitRealFunction uniform_unit_real_fn = [&]() {
    return planning_space->random_source().DrawUniformUnitReal(
        kMainThreadNumber);
  };

  const BiRRTSelectSampleTypeFunction<StateType> select_sample_type_fn =
      MakeUniformRandomBiRRTSelectSampleTypeFunction<StateType>(
          uniform_unit_real_fn, parameters.tree_sampling_bias);

  const BiRRTTreeSamplingFunction<StateType> tree_sampling_fn =
      MakeUniformRandomBiRRTTreeSamplingFunction<StateType>(
          uniform_unit_real_fn);

  const BiRRTSelectActiveTreeFunction<StateType> select_active_tree_fn =
      MakeUniformRandomBiRRTSelectActiveTreeFunction<StateType>(
          uniform_unit_real_fn, parameters.p_switch_trees);

  // Define our own termination function, which checks if the time/tree growth
  // limit has been reached.
  internal::SerialTerminationHelper termination_helper(
      parameters.time_limit, parameters.tree_growth_limit);

  const BiRRTTerminationCheckFunction termination_check_fn =
      [&](const int64_t start_tree_size, const int64_t goal_tree_size) {
    return termination_helper.CheckBiRRTTermination(
        start_tree_size, goal_tree_size);
  };

  // Call the planner
  drake::log()->log(
      parameters.planner_log_level, "Calling BiRRTPlanSinglePath()...");

  const auto result = BiRRTPlanSinglePath(
      start_tree, goal_tree, select_sample_type_fn, sampling_fn,
      tree_sampling_fn, nearest_neighbor_fn, propagation_fn, {},
      states_connected_fn, {}, select_active_tree_fn, termination_check_fn);

  auto combined_statistics = result.Statistics();
  combined_statistics.merge(propagation_statistics);
  drake::log()->log(
      parameters.planner_log_level,
      "BiRRT statistics {}",
      common_robotics_utilities::print::Print(combined_statistics));

  if (result.Path().empty()) {
    drake::log()->warn("BiRRT failed to plan a path");
    return PathPlanningResult<StateType>(internal::GetPlanningErrors(
        parameters.time_limit, combined_statistics.at("planning_time"),
        parameters.tree_growth_limit,
        static_cast<int>(combined_statistics.at("total_states"))));
  } else {
    const double path_length = planning_space->CalcPathLength(result.Path());
    drake::log()->log(
        parameters.planner_log_level,
        "BiRRT found path of length {} with {} states",
        path_length, result.Path().size());
    return PathPlanningResult<StateType>(result.Path(), path_length);
  }
}

}  // namespace planning
}  // namespace anzu

ANZU_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_PLANNING_STATE_TYPES(
    class ::anzu::planning::BiRRTPlanner)
