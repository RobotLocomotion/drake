#include "drake/planning/sampling_based/dev/rrt_planner.h"

#include <map>
#include <string>

#include <common_robotics_utilities/parallelism.hpp>
#include <common_robotics_utilities/print.hpp>
#include <common_robotics_utilities/simple_knearest_neighbors.hpp>
#include <common_robotics_utilities/simple_rrt_planner.hpp>

#include "drake/common/drake_throw.h"
#include "drake/common/text_logging.h"
#include "drake/planning/sampling_based/dev/rrt_internal.h"

namespace drake {
namespace planning {
using common_robotics_utilities::parallelism::DegreeOfParallelism;
using common_robotics_utilities::simple_rrt_planner::ForwardPropagation;
using common_robotics_utilities::simple_rrt_planner::
    MakeLinearRRTNearestNeighborsFunction;
using common_robotics_utilities::simple_rrt_planner::
    MakeStateAndGoalsSamplingFunction;
using common_robotics_utilities::simple_rrt_planner::
    RRTCheckGoalReachedFunction;
using common_robotics_utilities::simple_rrt_planner::
    RRTForwardPropagationFunction;
using common_robotics_utilities::simple_rrt_planner::RRTPlanSinglePath;
using common_robotics_utilities::simple_rrt_planner::
    RRTTerminationCheckFunction;
using common_robotics_utilities::simple_rrt_planner::SamplingFunction;
using common_robotics_utilities::simple_rrt_planner::SimpleRRTPlannerTree;

// Functions that take thread numbers are only ever called by the RRT planner
// in the main thread, to be consistent we specify thread_number=0 everywhere.
constexpr int kMainThreadNumber = 0;

namespace {
DegreeOfParallelism ToCRU(const Parallelism parallelism) {
  return DegreeOfParallelism(parallelism.num_threads());
}

template <typename StateType>
PathPlanningResult<StateType> DoPlan(
    const std::vector<StateType>& valid_starts,
    const typename RRTPlanner<StateType>::Parameters& parameters,
    PlanningSpace<StateType>* const planning_space,
    internal::GoalStateKeeper<StateType>* const goal_state_keeper,
    const GoalChecker<StateType>* const goal_checker) {
  // EITHER a goal state keeper OR goal checker must be provided.
  DRAKE_THROW_UNLESS((goal_state_keeper != nullptr) !=
                     (goal_checker != nullptr));

  // Use the generator associated with thread 0.
  auto& generator =
      planning_space->random_source().generator(kMainThreadNumber);

  // Sampling function.
  SamplingFunction<StateType> state_sampling_fn;
  if (goal_state_keeper != nullptr) {
    state_sampling_fn = [&]() {
      if (DrawUniformUnitReal(&generator) > parameters.goal_sampling_bias) {
        return planning_space->SampleState(kMainThreadNumber);
      } else {
        return goal_state_keeper->Sample(&generator, kMainThreadNumber);
      }
    };
  } else {
    state_sampling_fn = [&]() {
      return planning_space->SampleState(kMainThreadNumber);
    };
  }

  // Statistics for edge propagation function.
  std::map<std::string, double> propagation_statistics;

  const RRTForwardPropagationFunction<StateType, StateType>
      forward_propagation_fn = [&](const StateType& nearest,
                                   const StateType& sample) {
        const std::vector<StateType> propagated_states =
            planning_space->PropagateForwards(
                nearest, sample, &propagation_statistics, kMainThreadNumber);
        return internal::MakeForwardPropagation(propagated_states);
      };

  // Goal check function.
  RRTCheckGoalReachedFunction<StateType> internal_goal_check_fn;
  if (goal_state_keeper != nullptr) {
    internal_goal_check_fn = [&](const StateType& candidate) {
      return goal_state_keeper->CheckGoalReached(candidate);
    };
  } else {
    internal_goal_check_fn = [&](const StateType& candidate) {
      return goal_checker->CheckGoalReached(candidate, kMainThreadNumber);
    };
  }

  // Assemble tree from start states.
  SimpleRRTPlannerTree<StateType> tree(valid_starts.size());
  for (const auto& start : valid_starts) {
    tree.AddNode(start);
  }

  // Define our own termination function, which checks if the time/tree growth
  // limit has been reached.
  internal::SerialTerminationHelper termination_helper(
      parameters.time_limit, parameters.tree_growth_limit);

  const RRTTerminationCheckFunction termination_check_fn =
      [&](const int64_t tree_size) {
        return termination_helper.CheckRRTTermination(tree_size);
      };

  drake::log()->log(parameters.planner_log_level,
                    "Calling RRTPlanSinglePath()...");
  const auto result = RRTPlanSinglePath(
      tree, state_sampling_fn,
      MakeLinearRRTNearestNeighborsFunction<
          StateType, SimpleRRTPlannerTree<StateType>, StateType>(
          [&](const StateType& tree_state, const StateType& sampled) {
            return planning_space->NearestNeighborDistanceForwards(tree_state,
                                                                   sampled);
          },
          ToCRU(parameters.nearest_neighbor_parallelism)),
      forward_propagation_fn, {}, internal_goal_check_fn, {},
      termination_check_fn);

  auto combined_statistics = result.Statistics();
  combined_statistics.merge(propagation_statistics);
  drake::log()->log(
      parameters.planner_log_level, "RRT statistics {}",
      common_robotics_utilities::print::Print(combined_statistics));

  if (result.Path().empty()) {
    drake::log()->warn("RRT failed to plan a path");
    return PathPlanningResult<StateType>(internal::GetPlanningErrors(
        parameters.time_limit, combined_statistics.at("planning_time"),
        parameters.tree_growth_limit,
        static_cast<int>(combined_statistics.at("total_states"))));
  } else {
    const double path_length = planning_space->CalcPathLength(result.Path());
    drake::log()->log(parameters.planner_log_level,
                      "RRT found path of length {} with {} states", path_length,
                      result.Path().size());
    return PathPlanningResult<StateType>(result.Path(), path_length);
  }
}

}  // namespace

template <typename StateType>
PathPlanningResult<StateType> RRTPlanner<StateType>::Plan(
    const StateType& start, const StateType& goal, const Parameters& parameters,
    PlanningSpace<StateType>* const planning_space) {
  return Plan(std::vector<StateType>{start}, std::vector<StateType>{goal},
              parameters, planning_space);
}

template <typename StateType>
PathPlanningResult<StateType> RRTPlanner<StateType>::PlanGoalSampling(
    const StateType& start, const GoalSampler<StateType>& goal_sampler,
    const Parameters& parameters,
    PlanningSpace<StateType>* const planning_space) {
  return PlanGoalSampling(std::vector<StateType>{start}, goal_sampler,
                          parameters, planning_space);
}

template <typename StateType>
PathPlanningResult<StateType> RRTPlanner<StateType>::PlanGoalCheck(
    const StateType& start, const GoalChecker<StateType>& goal_checker,
    const Parameters& parameters,
    PlanningSpace<StateType>* const planning_space) {
  return PlanGoalCheck(std::vector<StateType>{start}, goal_checker, parameters,
                       planning_space);
}

template <typename StateType>
PathPlanningResult<StateType> RRTPlanner<StateType>::Plan(
    const std::vector<StateType>& starts, const std::vector<StateType>& goals,
    const Parameters& parameters,
    PlanningSpace<StateType>* const planning_space) {
  DRAKE_THROW_UNLESS(parameters.goal_sampling_bias > 0.0);
  DRAKE_THROW_UNLESS(parameters.goal_sampling_bias < 1.0);
  DRAKE_THROW_UNLESS(parameters.goal_tolerance >= 0.0);
  DRAKE_THROW_UNLESS(planning_space != nullptr);

  internal::ValidateRRTTerminationConditions(parameters, "RRTPlanner");

  const auto& [valid_starts, valid_goals, errors] =
      planning_space->ExtractValidStartsAndGoals(starts, goals,
                                                 kMainThreadNumber);
  if (!errors.empty()) {
    return PathPlanningResult<StateType>(errors);
  }

  internal::StaticGoalStateKeeper<StateType> goal_keeper(
      planning_space, parameters.goal_tolerance, valid_goals);

  return DoPlan<StateType>(valid_starts, parameters, planning_space,
                           &goal_keeper, nullptr /* goal_checker */);
}

template <typename StateType>
PathPlanningResult<StateType> RRTPlanner<StateType>::PlanGoalSampling(
    const std::vector<StateType>& starts,
    const GoalSampler<StateType>& goal_sampler, const Parameters& parameters,
    PlanningSpace<StateType>* const planning_space) {
  DRAKE_THROW_UNLESS(parameters.goal_sampling_bias > 0.0);
  DRAKE_THROW_UNLESS(parameters.goal_sampling_bias < 1.0);
  DRAKE_THROW_UNLESS(parameters.p_goal_sample_is_new > 0.0);
  DRAKE_THROW_UNLESS(parameters.p_goal_sample_is_new <= 1.0);
  DRAKE_THROW_UNLESS(parameters.goal_tolerance >= 0.0);
  DRAKE_THROW_UNLESS(planning_space != nullptr);

  internal::ValidateRRTTerminationConditions(parameters, "RRTPlanner");

  const auto& [valid_starts, errors] =
      planning_space->ExtractValidStarts(starts, kMainThreadNumber);
  if (!errors.empty()) {
    return PathPlanningResult<StateType>(errors);
  }

  internal::SerialDynamicGoalStateKeeper<StateType> goal_keeper(
      planning_space, parameters.goal_tolerance, &goal_sampler,
      parameters.p_goal_sample_is_new);

  return DoPlan<StateType>(valid_starts, parameters, planning_space,
                           &goal_keeper, nullptr /* goal_checker */);
}

template <typename StateType>
PathPlanningResult<StateType> RRTPlanner<StateType>::PlanGoalCheck(
    const std::vector<StateType>& starts,
    const GoalChecker<StateType>& goal_checker, const Parameters& parameters,
    PlanningSpace<StateType>* const planning_space) {
  DRAKE_THROW_UNLESS(planning_space != nullptr);

  internal::ValidateRRTTerminationConditions(parameters, "RRTPlanner");

  const auto& [valid_starts, errors] =
      planning_space->ExtractValidStarts(starts, kMainThreadNumber);
  if (!errors.empty()) {
    return PathPlanningResult<StateType>(errors);
  }

  return DoPlan<StateType>(valid_starts, parameters, planning_space,
                           nullptr /* goal_keeper */, &goal_checker);
}

}  // namespace planning
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_PLANNING_STATE_TYPES(
    class ::drake::planning::RRTPlanner)
