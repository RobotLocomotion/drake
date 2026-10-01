#include "drake/planning/sampling_based/dev/parallel_birrt_planner.h"

#include <algorithm>
#include <atomic>
#include <limits>
#include <map>
#include <memory>
#include <string>
#include <thread>
#include <utility>

#include <common_robotics_utilities/print.hpp>
#include <common_robotics_utilities/simple_knearest_neighbors.hpp>
#include <common_robotics_utilities/simple_rrt_planner.hpp>

#include "drake/common/drake_throw.h"
#include "drake/common/text_logging.h"
#include "drake/planning/sampling_based/dev/parallel_rrt_planner_tree.h"
#include "drake/planning/sampling_based/dev/rrt_internal.h"

namespace drake {
namespace planning {
using common_robotics_utilities::simple_rrt_planner::BiRRTActiveTreeType;
using common_robotics_utilities::simple_rrt_planner::
    BiRRTGoalBridgeCallbackFunction;
using common_robotics_utilities::simple_rrt_planner::
    BiRRTNearestNeighborFunction;
using common_robotics_utilities::simple_rrt_planner::BiRRTPlanMultiPath;
using common_robotics_utilities::simple_rrt_planner::BiRRTPropagationFunction;
using common_robotics_utilities::simple_rrt_planner::
    BiRRTSelectActiveTreeFunction;
using common_robotics_utilities::simple_rrt_planner::
    BiRRTSelectSampleTypeFunction;
using common_robotics_utilities::simple_rrt_planner::
    BiRRTStatesConnectedFunction;
using common_robotics_utilities::simple_rrt_planner::
    BiRRTTerminationCheckFunction;
using common_robotics_utilities::simple_rrt_planner::BiRRTTreeSamplingFunction;
using common_robotics_utilities::simple_rrt_planner::ForwardPropagation;
using common_robotics_utilities::simple_rrt_planner::
    MakeUniformRandomBiRRTSelectActiveTreeFunction;
using common_robotics_utilities::simple_rrt_planner::
    MakeUniformRandomBiRRTSelectSampleTypeFunction;
using common_robotics_utilities::simple_rrt_planner::
    MakeUniformRandomBiRRTTreeSamplingFunction;
using common_robotics_utilities::simple_rrt_planner::
    MultipleSolutionPlanningResults;
using common_robotics_utilities::simple_rrt_planner::SamplingFunction;
using common_robotics_utilities::simple_rrt_planner::
    SingleSolutionPlanningResults;
using common_robotics_utilities::utility::UniformUnitRealFunction;

// For functions that are only ever called by the main thread, we specify
// thread_number=0 to be consistent.
constexpr int kMainThreadNumber = 0;

namespace {
/// Parallel worker that handles per-worker shared state (e.g. collision
/// checking context, sampler, constraints) and runs the per-worker BiRRT
/// planner.
template <typename StateType>
class ParallelBiRRTWorker {
 public:
  /// Constructor.
  /// @param worker_num Number of worker, used in log messages to identify the
  /// worker responsible for the message.
  /// @param planning_space Planning space to use.
  /// @param solution_found Used to track when a solution is found, shared
  /// between all workers.
  ParallelBiRRTWorker(int32_t worker_num,
                      PlanningSpace<StateType>* planning_space,
                      std::atomic<bool>* const solution_found)
      : worker_num_(worker_num),
        planning_space_(planning_space),
        solution_found_(solution_found) {
    DRAKE_THROW_UNLESS(planning_space_ != nullptr);
    DRAKE_THROW_UNLESS(solution_found_ != nullptr);
  }

  const SingleSolutionPlanningResults<StateType>& solution() const {
    return solution_;
  }

  void Plan(
      const typename ParallelBiRRTPlanner<StateType>::Parameters& parameters,
      internal::ParallelRRTPlannerTree<StateType>* const start_tree,
      internal::ParallelRRTPlannerTree<StateType>* const goal_tree) {
    DRAKE_THROW_UNLESS(start_tree != nullptr);
    DRAKE_THROW_UNLESS(goal_tree != nullptr);

    // Build helper functions. For operations performed on/from the start tree,
    // we use the "forwards" methods provided by the planning space; for
    // operations on/from the goal tree, we use the "backwards" methods instead.

    // Sampling function.
    const SamplingFunction<StateType> sampling_fn = [&]() {
      return mutable_planning_space().SampleState(worker_num_);
    };

    // Nearest-neighbor function.
    const BiRRTNearestNeighborFunction<
        StateType, internal::ParallelRRTPlannerTree<StateType>>
        nearest_neighbor_fn =
            [&](const internal::ParallelRRTPlannerTree<StateType>& tree,
                const StateType& sample,
                const BiRRTActiveTreeType active_tree_type) {
              switch (active_tree_type) {
                case BiRRTActiveTreeType::START_TREE:
                  return internal::GetParallelRRTNearestNeighbor<StateType>(
                      tree, sample,
                      [&](const StateType& from, const StateType& to) {
                        return planning_space().NearestNeighborDistanceForwards(
                            from, to);
                      },
                      parameters.nearest_neighbor_parallelism);
                case BiRRTActiveTreeType::GOAL_TREE:
                  return internal::GetParallelRRTNearestNeighbor<StateType>(
                      tree, sample,
                      [&](const StateType& from, const StateType& to) {
                        return planning_space()
                            .NearestNeighborDistanceBackwards(from, to);
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
              propagated_states = mutable_planning_space().PropagateForwards(
                  nearest, sampled, &propagation_statistics, worker_num_);
              break;
            case BiRRTActiveTreeType::GOAL_TREE:
              propagated_states = mutable_planning_space().PropagateBackwards(
                  nearest, sampled, &propagation_statistics, worker_num_);
              break;
          }

          return internal::MakeForwardPropagation(propagated_states);
        };

    // State-state connection check function.
    const BiRRTStatesConnectedFunction<StateType> states_connected_fn =
        [&](const StateType& from, const StateType& to,
            const BiRRTActiveTreeType active_tree_type) {
          double distance = 0.0;
          switch (active_tree_type) {
            case BiRRTActiveTreeType::START_TREE:
              distance = planning_space().StateDistanceForwards(from, to);
              break;
            case BiRRTActiveTreeType::GOAL_TREE:
              distance = planning_space().StateDistanceBackwards(from, to);
              break;
          }
          return distance <= parameters.connection_tolerance;
        };

    // Define our own solution-found callback to check when the first path is
    // found.
    const BiRRTGoalBridgeCallbackFunction<
        StateType, internal::ParallelRRTPlannerTree<StateType>>
        solution_found_fn = [&](internal::ParallelRRTPlannerTree<StateType>&,
                                int64_t,
                                internal::ParallelRRTPlannerTree<StateType>&,
                                int64_t, BiRRTActiveTreeType) {
          solution_found_->store(true);
        };

    // Define our own termination function, which checks if the time/tree growth
    // limit has been reached or if any worker has found a solution.
    internal::ParallelTerminationHelper termination_helper(
        parameters.time_limit, parameters.tree_growth_limit, solution_found_);

    const BiRRTTerminationCheckFunction termination_check_fn =
        [&](const int64_t start_tree_size, const int64_t goal_tree_size) {
          return termination_helper.CheckBiRRTTermination(start_tree_size,
                                                          goal_tree_size);
        };

    const UniformUnitRealFunction uniform_unit_real_fn = [&]() {
      return mutable_planning_space().random_source().DrawUniformUnitReal(
          worker_num_);
    };

    const BiRRTSelectSampleTypeFunction<
        StateType, internal::ParallelRRTPlannerTree<StateType>>
        select_sample_type_fn = MakeUniformRandomBiRRTSelectSampleTypeFunction<
            StateType, internal::ParallelRRTPlannerTree<StateType>>(
            uniform_unit_real_fn, parameters.tree_sampling_bias);

    const BiRRTTreeSamplingFunction<StateType,
                                    internal::ParallelRRTPlannerTree<StateType>>
        tree_sampling_fn = MakeUniformRandomBiRRTTreeSamplingFunction<
            StateType, internal::ParallelRRTPlannerTree<StateType>>(
            uniform_unit_real_fn);

    const BiRRTSelectActiveTreeFunction<
        StateType, internal::ParallelRRTPlannerTree<StateType>>
        select_active_tree_fn = MakeUniformRandomBiRRTSelectActiveTreeFunction<
            StateType, internal::ParallelRRTPlannerTree<StateType>>(
            uniform_unit_real_fn, parameters.p_switch_trees);

    drake::log()->log(parameters.planner_log_level,
                      "[Worker {}] Starting BiRRT planner...", worker_num_);
    // Note: we call BiRRTPlanMultiPath rather than BiRRTPlanSinglePath to
    // avoid having two layers of solution-found checks.
    const MultipleSolutionPlanningResults<StateType> birrt_result =
        BiRRTPlanMultiPath(*start_tree, *goal_tree, select_sample_type_fn,
                           sampling_fn, tree_sampling_fn, nearest_neighbor_fn,
                           propagation_fn, {}, states_connected_fn,
                           solution_found_fn, select_active_tree_fn,
                           termination_check_fn);

    drake::log()->log(parameters.planner_log_level,
                      "[Worker {}] Collecting result...", worker_num_);
    if (birrt_result.Paths().size() > 0) {
      // Note: a given worker will only ever produce a single path, as it will
      // stop planning after the first solution is found.
      solution_ = SingleSolutionPlanningResults<StateType>(
          birrt_result.Paths().at(0), birrt_result.Statistics());
    } else {
      solution_ =
          SingleSolutionPlanningResults<StateType>(birrt_result.Statistics());
    }
  }

 private:
  const PlanningSpace<StateType>& planning_space() const {
    return *planning_space_;
  }

  PlanningSpace<StateType>& mutable_planning_space() {
    return *planning_space_;
  }

  const int32_t worker_num_;
  PlanningSpace<StateType>* const planning_space_;

  std::atomic<bool>* const solution_found_;

  SingleSolutionPlanningResults<StateType> solution_;
};

template <typename StateType>
PathPlanningResult<StateType> GetPathPlanningResult(
    const std::vector<SingleSolutionPlanningResults<StateType>>& solutions,
    const PlanningSpace<StateType>& planning_space,
    const typename ParallelBiRRTPlanner<StateType>::Parameters& parameters) {
  // Merge statistics.
  std::map<std::string, double> merged_statistics;
  for (const auto& solution : solutions) {
    for (const auto& [key, value] : solution.Statistics()) {
      if (key == "total_states") {
        // total_states is a property of the shared tree, not per-planner, and
        // should not be combined.
        merged_statistics[key] = value;
      } else if (key == "planning_time") {
        // Record the longest planning time of any worker.
        merged_statistics[key] = std::max(value, merged_statistics[key]);
      } else {
        merged_statistics[key] += value;
      }
    }
  }

  drake::log()->log(parameters.planner_log_level, "ParallelBiRRT statistics {}",
                    common_robotics_utilities::print::Print(merged_statistics));

  // Return the best solution.
  int32_t best_solution_index = -1;
  double best_solution_length = std::numeric_limits<double>::infinity();

  for (size_t solution_num = 0; solution_num < solutions.size();
       ++solution_num) {
    const auto& solution = solutions.at(solution_num);
    if (solution.Path().size() > 0) {
      const double solution_length =
          planning_space.CalcPathLength(solution.Path());
      if (solution_length < best_solution_length) {
        best_solution_index = static_cast<int32_t>(solution_num);
        best_solution_length = solution_length;
      }
    }
  }

  if (best_solution_index < 0) {
    drake::log()->warn("ParallelBiRRT failed to plan a path");
    return PathPlanningResult<StateType>(internal::GetPlanningErrors(
        parameters.time_limit, merged_statistics.at("planning_time"),
        parameters.tree_growth_limit,
        static_cast<int>(merged_statistics.at("total_states"))));
  } else {
    const auto& best_solution_path = solutions.at(best_solution_index).Path();
    drake::log()->log(parameters.planner_log_level,
                      "ParallelBiRRT found path of length {} with {} states",
                      best_solution_length, best_solution_path.size());
    return PathPlanningResult<StateType>(best_solution_path,
                                         best_solution_length);
  }
}

}  // namespace

template <typename StateType>
PathPlanningResult<StateType> ParallelBiRRTPlanner<StateType>::Plan(
    const StateType& start, const StateType& goal, const Parameters& parameters,
    PlanningSpace<StateType>* const planning_space) {
  return Plan(std::vector<StateType>{start}, std::vector<StateType>{goal},
              parameters, planning_space);
}

template <typename StateType>
PathPlanningResult<StateType> ParallelBiRRTPlanner<StateType>::Plan(
    const std::vector<StateType>& starts, const std::vector<StateType>& goals,
    const Parameters& parameters,
    PlanningSpace<StateType>* const planning_space) {
  DRAKE_THROW_UNLESS(parameters.tree_sampling_bias > 0.0);
  DRAKE_THROW_UNLESS(parameters.tree_sampling_bias < 1.0);
  DRAKE_THROW_UNLESS(parameters.p_switch_trees > 0.0);
  DRAKE_THROW_UNLESS(parameters.p_switch_trees <= 1.0);
  DRAKE_THROW_UNLESS(parameters.connection_tolerance >= 0.0);
  DRAKE_THROW_UNLESS(parameters.num_workers > 0);
  DRAKE_THROW_UNLESS(parameters.initial_tree_capacity >= 0);
  DRAKE_THROW_UNLESS(planning_space != nullptr);

  internal::ValidateBiRRTTerminationConditions(parameters,
                                               "ParallelBiRRTPlanner");

  DRAKE_THROW_UNLESS(parameters.num_workers <=
                     planning_space->parallelism().num_threads());

  const int hardware_concurrency = std::thread::hardware_concurrency();
  const int num_nn_threads =
      parameters.nearest_neighbor_parallelism.num_threads();
  const int requested_concurrency = parameters.num_workers * num_nn_threads;
  if (requested_concurrency > hardware_concurrency) {
    drake::log()->warn(
        "Requested concurrency {} ({} workers x {} NN threads) exceeds hardware"
        " concurrency {}",
        requested_concurrency, parameters.num_workers, num_nn_threads,
        hardware_concurrency);
  }

  const auto& [valid_starts, valid_goals, errors] =
      planning_space->ExtractValidStartsAndGoals(starts, goals,
                                                 kMainThreadNumber);
  if (!errors.empty()) {
    return PathPlanningResult<StateType>(errors);
  }

  // Assemble starts & goals.
  internal::ParallelRRTPlannerTree<StateType> start_tree(
      parameters.initial_tree_capacity);
  for (const StateType& start : valid_starts) {
    start_tree.AddNode(start);
  }
  internal::ParallelRRTPlannerTree<StateType> goal_tree(
      parameters.initial_tree_capacity);
  for (const StateType& goal : valid_goals) {
    goal_tree.AddNode(goal);
  }

  // Assemble workers.
  drake::log()->log(parameters.planner_log_level,
                    "Building {} ParallelBiRRT workers...",
                    parameters.num_workers);
  std::atomic<bool> solution_found(false);

  std::vector<ParallelBiRRTWorker<StateType>> workers;
  for (int32_t worker_num = 0; worker_num < parameters.num_workers;
       ++worker_num) {
    workers.emplace_back(worker_num, planning_space, &solution_found);
  }

  // Start planners.
  drake::log()->log(parameters.planner_log_level,
                    "Dispatching ParallelBiRRT planner threads...");
  std::vector<std::thread> worker_threads;
  for (int32_t worker_num = 0; worker_num < parameters.num_workers;
       ++worker_num) {
    ParallelBiRRTWorker<StateType>& worker = workers.at(worker_num);
    const auto worker_thread_fn = [&]() {
      worker.Plan(parameters, &start_tree, &goal_tree);
    };
    worker_threads.emplace_back(worker_thread_fn);
  }

  // Wait for planners to finish.
  drake::log()->log(parameters.planner_log_level,
                    "Waiting for ParallelBiRRT planner threads to complete...");
  for (auto& worker_thread : worker_threads) {
    worker_thread.join();
  }

  // Collect solutions.
  std::vector<SingleSolutionPlanningResults<StateType>> solutions;
  for (const ParallelBiRRTWorker<StateType>& worker : workers) {
    solutions.push_back(worker.solution());
  }

  return GetPathPlanningResult(solutions, *planning_space, parameters);
}

}  // namespace planning
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_PLANNING_STATE_TYPES(
    class ::drake::planning::ParallelBiRRTPlanner)
