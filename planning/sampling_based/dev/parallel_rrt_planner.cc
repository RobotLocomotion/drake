#include "drake/planning/sampling_based/dev/parallel_rrt_planner.h"

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
using common_robotics_utilities::simple_rrt_planner::ForwardPropagation;
using common_robotics_utilities::simple_rrt_planner::
    MultipleSolutionPlanningResults;
using common_robotics_utilities::simple_rrt_planner::
    RRTCheckGoalReachedFunction;
using common_robotics_utilities::simple_rrt_planner::
    RRTForwardPropagationFunction;
using common_robotics_utilities::simple_rrt_planner::
    RRTGoalReachedCallbackFunction;
using common_robotics_utilities::simple_rrt_planner::RRTNearestNeighborFunction;
using common_robotics_utilities::simple_rrt_planner::RRTPlanMultiPath;
using common_robotics_utilities::simple_rrt_planner::
    RRTTerminationCheckFunction;
using common_robotics_utilities::simple_rrt_planner::SamplingFunction;
using common_robotics_utilities::simple_rrt_planner::
    SingleSolutionPlanningResults;

// For functions that are only ever called by the main thread, we specify
// thread_number=0 to be consistent.
constexpr int kMainThreadNumber = 0;

namespace {
/// Parallel worker that handles per-worker shared state (e.g. collision
/// checking context, sampler, constraints) and runs the per-worker RRT planner.
template <typename StateType>
class ParallelRRTWorker {
 public:
  /// Constructor.
  /// @param worker_num Number of worker, used in log messages to identify the
  /// worker responsible for the message.
  /// @param planning_space Planning space to use. Planning space is reseeded
  /// with the provided seed.
  /// @param solution_found Used to track when a solution is found, shared
  /// between all workers.
  /// @param prng_seed Seed for internal generator, also used to reseed the
  /// planning space.
  ParallelRRTWorker(int32_t worker_num,
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

  void PlanGoalKeeper(
      const typename ParallelRRTPlanner<StateType>::Parameters& parameters,
      internal::ParallelRRTPlannerTree<StateType>* const tree,
      internal::GoalStateKeeper<StateType>* const goal_state_keeper) {
    DRAKE_THROW_UNLESS(tree != nullptr);
    DRAKE_THROW_UNLESS(goal_state_keeper != nullptr);

    auto& generator =
        mutable_planning_space().random_source().generator(worker_num_);

    // Sampling function.
    const SamplingFunction<StateType> sampling_fn = [&]() {
      if (DrawUniformUnitReal(&generator) > parameters.goal_sampling_bias) {
        return mutable_planning_space().SampleState(worker_num_);
      } else {
        return goal_state_keeper->Sample(&generator, worker_num_);
      }
    };

    // Goal check function.
    const RRTCheckGoalReachedFunction<StateType> goal_check_fn =
        [&](const StateType& candidate) {
          return goal_state_keeper->CheckGoalReached(candidate);
        };

    return DoPlan(parameters, tree, sampling_fn, goal_check_fn);
  }

  void PlanGoalChecker(
      const typename ParallelRRTPlanner<StateType>::Parameters& parameters,
      internal::ParallelRRTPlannerTree<StateType>* const tree,
      const GoalChecker<StateType>& goal_checker) {
    DRAKE_THROW_UNLESS(tree != nullptr);

    // Sampling function.
    const SamplingFunction<StateType> sampling_fn = [&]() {
      return mutable_planning_space().SampleState(worker_num_);
    };

    // Goal check function.
    const RRTCheckGoalReachedFunction<StateType> goal_check_fn =
        [&](const StateType& candidate) {
          return goal_checker.CheckGoalReached(candidate, worker_num_);
        };

    return DoPlan(parameters, tree, sampling_fn, goal_check_fn);
  }

 private:
  void DoPlan(
      const typename ParallelRRTPlanner<StateType>::Parameters& parameters,
      internal::ParallelRRTPlannerTree<StateType>* const tree,
      const SamplingFunction<StateType>& sampling_fn,
      const RRTCheckGoalReachedFunction<StateType>& goal_check_fn) {
    // Nearest-neighbor function.
    const RRTNearestNeighborFunction<
        StateType, internal::ParallelRRTPlannerTree<StateType>>
        nearest_neighbor_fn =
            [&](const internal::ParallelRRTPlannerTree<StateType>& planner_tree,
                const StateType& sample) {
              return internal::GetParallelRRTNearestNeighbor<StateType>(
                  planner_tree, sample,
                  [&](const StateType& from, const StateType& to) {
                    return planning_space().NearestNeighborDistanceForwards(
                        from, to);
                  },
                  parameters.nearest_neighbor_parallelism);
            };

    // Statistics for edge propagation function.
    std::map<std::string, double> propagation_statistics;

    const RRTForwardPropagationFunction<StateType, StateType>
        forward_propagation_fn =
            [&](const StateType& nearest, const StateType& sample) {
              const std::vector<StateType> propagated_states =
                  mutable_planning_space().PropagateForwards(
                      nearest, sample, &propagation_statistics, worker_num_);
              return internal::MakeForwardPropagation(propagated_states);
            };

    // Define our own solution-found callback to check when the first path is
    // found.
    const RRTGoalReachedCallbackFunction<
        StateType, internal::ParallelRRTPlannerTree<StateType>>
        solution_found_fn =
            [&](internal::ParallelRRTPlannerTree<StateType>&, int64_t) {
              solution_found_->store(true);
            };

    // Define our own termination function, which checks if the time/tree growth
    // limit has been reached or if any worker has found a solution.
    internal::ParallelTerminationHelper termination_helper(
        parameters.time_limit, parameters.tree_growth_limit, solution_found_);

    const RRTTerminationCheckFunction termination_check_fn =
        [&](const int64_t tree_size) {
          return termination_helper.CheckRRTTermination(tree_size);
        };

    drake::log()->log(parameters.planner_log_level,
                      "[Worker {}] Starting RRT planner...", worker_num_);
    // Note: we call RRTPlanMultiPath rather than RRTPlanSinglePath to
    // avoid having two layers of solution-found checks.
    const MultipleSolutionPlanningResults<StateType> rrt_result =
        RRTPlanMultiPath(*tree, sampling_fn, nearest_neighbor_fn,
                         forward_propagation_fn, {}, goal_check_fn,
                         solution_found_fn, termination_check_fn);

    drake::log()->log(parameters.planner_log_level,
                      "[Worker {}] Collecting result...", worker_num_);
    if (rrt_result.Paths().size() > 0) {
      // Note: a given worker will only ever produce a single path, as it will
      // stop planning after the first solution is found.
      solution_ = SingleSolutionPlanningResults<StateType>(
          rrt_result.Paths().at(0), rrt_result.Statistics());
    } else {
      solution_ =
          SingleSolutionPlanningResults<StateType>(rrt_result.Statistics());
    }
  }

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
    const typename ParallelRRTPlanner<StateType>::Parameters& parameters) {
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

  drake::log()->log(parameters.planner_log_level, "ParallelRRT statistics {}",
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
    drake::log()->warn("ParallelRRT failed to plan a path");
    return PathPlanningResult<StateType>(internal::GetPlanningErrors(
        parameters.time_limit, merged_statistics.at("planning_time"),
        parameters.tree_growth_limit,
        static_cast<int>(merged_statistics.at("total_states"))));
  } else {
    const auto& best_solution_path = solutions.at(best_solution_index).Path();
    drake::log()->log(parameters.planner_log_level,
                      "ParallelRRT found path of length {} with {} states",
                      best_solution_length, best_solution_path.size());
    return PathPlanningResult<StateType>(best_solution_path,
                                         best_solution_length);
  }
}

template <typename StateType>
PathPlanningResult<StateType> DoPlan(
    const std::vector<StateType>& valid_starts,
    const typename ParallelRRTPlanner<StateType>::Parameters& parameters,
    PlanningSpace<StateType>* const planning_space,
    internal::GoalStateKeeper<StateType>* const goal_state_keeper,
    const GoalChecker<StateType>* const goal_checker) {
  // EITHER a goal state keeper OR goal checker must be provided.
  DRAKE_THROW_UNLESS((goal_state_keeper != nullptr) !=
                     (goal_checker != nullptr));

  // Assemble tree from start states.
  internal::ParallelRRTPlannerTree<StateType> tree(
      parameters.initial_tree_capacity);
  for (const StateType& start : valid_starts) {
    tree.AddNode(start);
  }

  // Assemble workers.
  drake::log()->log(parameters.planner_log_level,
                    "Building {} ParallelRRT workers...",
                    parameters.num_workers);
  std::atomic<bool> solution_found(false);

  std::vector<ParallelRRTWorker<StateType>> workers;
  for (int32_t worker_num = 0; worker_num < parameters.num_workers;
       ++worker_num) {
    workers.emplace_back(worker_num, planning_space, &solution_found);
  }

  // Start planners.
  drake::log()->log(parameters.planner_log_level,
                    "Dispatching ParallelRRT planner threads...");
  std::vector<std::thread> worker_threads;
  for (int32_t worker_num = 0; worker_num < parameters.num_workers;
       ++worker_num) {
    ParallelRRTWorker<StateType>& worker = workers.at(worker_num);
    const auto worker_thread_fn = [&]() {
      if (goal_state_keeper != nullptr) {
        worker.PlanGoalKeeper(parameters, &tree, goal_state_keeper);
      } else {
        worker.PlanGoalChecker(parameters, &tree, *goal_checker);
      }
    };
    worker_threads.emplace_back(worker_thread_fn);
  }

  // Wait for planners to finish.
  drake::log()->log(parameters.planner_log_level,
                    "Waiting for ParallelRRT planner threads to complete...");
  for (auto& worker_thread : worker_threads) {
    worker_thread.join();
  }

  // Collect solutions.
  std::vector<SingleSolutionPlanningResults<StateType>> solutions;
  for (const ParallelRRTWorker<StateType>& worker : workers) {
    solutions.push_back(worker.solution());
  }

  return GetPathPlanningResult(solutions, *planning_space, parameters);
}

template <typename StateType>
void CheckRequestedConcurrency(
    const typename ParallelRRTPlanner<StateType>::Parameters& parameters,
    const PlanningSpace<StateType>& planning_space) {
  DRAKE_THROW_UNLESS(parameters.num_workers <=
                     planning_space.parallelism().num_threads());

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
}
}  // namespace

template <typename StateType>
PathPlanningResult<StateType> ParallelRRTPlanner<StateType>::Plan(
    const StateType& start, const StateType& goal, const Parameters& parameters,
    PlanningSpace<StateType>* const planning_space) {
  return Plan(std::vector<StateType>{start}, std::vector<StateType>{goal},
              parameters, planning_space);
}

template <typename StateType>
PathPlanningResult<StateType> ParallelRRTPlanner<StateType>::PlanGoalSampling(
    const StateType& start, const GoalSampler<StateType>& goal_sampler,
    const Parameters& parameters,
    PlanningSpace<StateType>* const planning_space) {
  return PlanGoalSampling(std::vector<StateType>{start}, goal_sampler,
                          parameters, planning_space);
}

template <typename StateType>
PathPlanningResult<StateType> ParallelRRTPlanner<StateType>::PlanGoalCheck(
    const StateType& start, const GoalChecker<StateType>& goal_checker,
    const Parameters& parameters,
    PlanningSpace<StateType>* const planning_space) {
  return PlanGoalCheck(std::vector<StateType>{start}, goal_checker, parameters,
                       planning_space);
}

template <typename StateType>
PathPlanningResult<StateType> ParallelRRTPlanner<StateType>::Plan(
    const std::vector<StateType>& starts, const std::vector<StateType>& goals,
    const Parameters& parameters,
    PlanningSpace<StateType>* const planning_space) {
  DRAKE_THROW_UNLESS(parameters.goal_sampling_bias > 0.0);
  DRAKE_THROW_UNLESS(parameters.goal_sampling_bias < 1.0);
  DRAKE_THROW_UNLESS(parameters.goal_tolerance >= 0.0);
  DRAKE_THROW_UNLESS(parameters.num_workers > 0);
  DRAKE_THROW_UNLESS(parameters.initial_tree_capacity >= 0);
  DRAKE_THROW_UNLESS(planning_space != nullptr);

  internal::ValidateRRTTerminationConditions(parameters, "ParallelRRTPlanner");

  CheckRequestedConcurrency<StateType>(parameters, *planning_space);

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
PathPlanningResult<StateType> ParallelRRTPlanner<StateType>::PlanGoalSampling(
    const std::vector<StateType>& starts,
    const GoalSampler<StateType>& goal_sampler, const Parameters& parameters,
    PlanningSpace<StateType>* const planning_space) {
  DRAKE_THROW_UNLESS(parameters.goal_sampling_bias > 0.0);
  DRAKE_THROW_UNLESS(parameters.goal_sampling_bias < 1.0);
  DRAKE_THROW_UNLESS(parameters.p_goal_sample_is_new > 0.0);
  DRAKE_THROW_UNLESS(parameters.p_goal_sample_is_new <= 1.0);
  DRAKE_THROW_UNLESS(parameters.goal_tolerance >= 0.0);
  DRAKE_THROW_UNLESS(parameters.num_workers > 0);
  DRAKE_THROW_UNLESS(parameters.initial_tree_capacity >= 0);
  DRAKE_THROW_UNLESS(planning_space != nullptr);

  internal::ValidateRRTTerminationConditions(parameters, "ParallelRRTPlanner");

  CheckRequestedConcurrency<StateType>(parameters, *planning_space);

  const auto& [valid_starts, errors] =
      planning_space->ExtractValidStarts(starts, kMainThreadNumber);
  if (!errors.empty()) {
    return PathPlanningResult<StateType>(errors);
  }

  internal::ParallelDynamicGoalStateKeeper<StateType> goal_keeper(
      planning_space, parameters.goal_tolerance, &goal_sampler,
      parameters.p_goal_sample_is_new);

  return DoPlan<StateType>(valid_starts, parameters, planning_space,
                           &goal_keeper, nullptr /* goal_checker */);
}

template <typename StateType>
PathPlanningResult<StateType> ParallelRRTPlanner<StateType>::PlanGoalCheck(
    const std::vector<StateType>& starts,
    const GoalChecker<StateType>& goal_checker, const Parameters& parameters,
    PlanningSpace<StateType>* const planning_space) {
  DRAKE_THROW_UNLESS(parameters.num_workers > 0);
  DRAKE_THROW_UNLESS(parameters.initial_tree_capacity >= 0);
  DRAKE_THROW_UNLESS(planning_space != nullptr);

  internal::ValidateRRTTerminationConditions(parameters, "ParallelRRTPlanner");

  CheckRequestedConcurrency<StateType>(parameters, *planning_space);

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
    class ::drake::planning::ParallelRRTPlanner)
