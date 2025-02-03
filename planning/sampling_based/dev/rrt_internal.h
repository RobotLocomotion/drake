#pragma once

#include <atomic>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <string_view>
#include <utility>
#include <vector>

#include <common_robotics_utilities/simple_rrt_planner.hpp>
#include <common_robotics_utilities/utility.hpp>

#include "planning/goal_sampler.h"
#include "planning/planning_space.h"

namespace anzu {
namespace planning {
namespace internal {
using common_robotics_utilities::utility::GetUniformRandomIndex;
using common_robotics_utilities::simple_rrt_planner::TimeoutCheckHelper;

// Helper function to assemble propagated states into a ForwardPropagation as
// consumed by RRT and BiRRT planners.
template <typename StateType>
common_robotics_utilities::simple_rrt_planner::ForwardPropagation<StateType>
MakeForwardPropagation(const std::vector<StateType>& propagated_states) {
  common_robotics_utilities::simple_rrt_planner::ForwardPropagation<StateType>
      forward_propagation;
  forward_propagation.reserve(propagated_states.size());
  int64_t relative_parent_index = -1;
  for (const auto& propagated_state : propagated_states) {
    forward_propagation.emplace_back(
        propagated_state, relative_parent_index);
    relative_parent_index++;
  }
  return forward_propagation;
}

// Interface for goal state keepers to implement.
template <typename StateType>
class GoalStateKeeper {
 public:
  GoalStateKeeper(
      const PlanningSpace<StateType>* planning_space,
      double goal_tolerance)
      : planning_space_(planning_space), goal_tolerance_(goal_tolerance) {
    DRAKE_THROW_UNLESS(planning_space_ != nullptr);
    DRAKE_THROW_UNLESS(goal_tolerance_ >= 0.0);
  }

  virtual ~GoalStateKeeper() = default;

  // Note that this method cannot be const as some implementations, e.g.
  // *DynamicGoalStateKeeper, mutate their internal state when sampling.
  StateType Sample(std::mt19937_64* generator, int thread_number) {
    DRAKE_THROW_UNLESS(generator != nullptr);
    return DoSample(generator, thread_number);
  }

  bool CheckGoalReached(const StateType& candidate) const {
    return DoCheckGoalReached(candidate);
  }

 protected:
  virtual StateType DoSample(std::mt19937_64* generator, int thread_number) = 0;

  virtual bool DoCheckGoalReached(const StateType& candidate) const = 0;

  const PlanningSpace<StateType>& planning_space() const {
    return *planning_space_;
  }

  double goal_tolerance() const { return goal_tolerance_; }

 private:
  const PlanningSpace<StateType>* const planning_space_;
  const double goal_tolerance_;
};

// Goal state keeper for static (fixed) goal states.
template <typename StateType>
class StaticGoalStateKeeper final : public GoalStateKeeper<StateType> {
 public:
  StaticGoalStateKeeper(
      const PlanningSpace<StateType>* planning_space, double goal_tolerance,
      std::vector<StateType> goal_states)
      : GoalStateKeeper<StateType>(planning_space, goal_tolerance),
        goal_states_(std::move(goal_states)) {
    DRAKE_THROW_UNLESS(goal_states_.size() > 0);
  }

 private:
  StateType DoSample(std::mt19937_64* generator, int) final {
    if (goal_states_.size() == 1) {
      return goal_states_.at(0);
    } else {
      return goal_states_.at(GetUniformRandomIndex(
          [generator]() { return DrawUniformUnitReal(generator); },
          goal_states_.size()));
    }
  }

  bool DoCheckGoalReached(const StateType& candidate) const final {
    for (const auto& goal : goal_states_) {
      if (this->planning_space().StateDistanceForwards(candidate, goal) <=
              this->goal_tolerance()) {
        return true;
      }
    }
    return false;
  }

  const std::vector<StateType> goal_states_;
};

// Goal state keeper for dynamic (sampled) goal states for use in serial RRT.
template <typename StateType>
class SerialDynamicGoalStateKeeper final : public GoalStateKeeper<StateType> {
 public:
  SerialDynamicGoalStateKeeper(
      const PlanningSpace<StateType>* planning_space, double goal_tolerance,
      const GoalSampler<StateType>* const goal_sampler,
      double p_goal_sample_is_new)
      : GoalStateKeeper<StateType>(planning_space, goal_tolerance),
        goal_sampler_(goal_sampler),
        p_goal_sample_is_new_(p_goal_sample_is_new) {
    DRAKE_THROW_UNLESS(goal_sampler_ != nullptr);
    DRAKE_THROW_UNLESS(p_goal_sample_is_new_ > 0.0);
    DRAKE_THROW_UNLESS(p_goal_sample_is_new_ <= 1.0);
  }

 private:
  StateType DoSample(std::mt19937_64* generator, int thread_number) final {
    const auto uniform_unit_real_fn =
        [generator]() { return DrawUniformUnitReal(generator); };

    if (uniform_unit_real_fn() > p_goal_sample_is_new_ &&
        goal_states_.size() > 0) {
      return goal_states_.at(GetUniformRandomIndex(
          uniform_unit_real_fn, goal_states_.size()));
    } else {
      const StateType goal_sample =
          goal_sampler_->Sample(generator, thread_number);
      goal_states_.emplace_back(goal_sample);
      return goal_sample;
    }
  }

  bool DoCheckGoalReached(const StateType& candidate) const final {
    for (const StateType& goal : goal_states_) {
      if (this->planning_space().StateDistanceForwards(candidate, goal) <=
              this->goal_tolerance()) {
        return true;
      }
    }
    return false;
  }

  const GoalSampler<StateType>* const goal_sampler_;
  const double p_goal_sample_is_new_;
  std::vector<StateType> goal_states_;
};

// Goal state keeper for dynamic (sampled) goal states for use in parallel RRT.
// This uses the same copy-on-grow approach as ParallelRRTPlannerTree to reduce
// the time spent locking on access to the shared goal states.
template <typename StateType>
class ParallelDynamicGoalStateKeeper final : public GoalStateKeeper<StateType> {
 public:
  ParallelDynamicGoalStateKeeper(
      const PlanningSpace<StateType>* planning_space, double goal_tolerance,
      const GoalSampler<StateType>* const goal_sampler,
      double p_goal_sample_is_new)
      : GoalStateKeeper<StateType>(planning_space, goal_tolerance),
        goal_sampler_(goal_sampler),
        p_goal_sample_is_new_(p_goal_sample_is_new)  {
    DRAKE_THROW_UNLESS(goal_sampler_ != nullptr);
    DRAKE_THROW_UNLESS(p_goal_sample_is_new_ > 0.0);
    DRAKE_THROW_UNLESS(p_goal_sample_is_new_ <= 1.0);
    // Initialize with a non-zero size to reduce frequency of grow operations.
    goal_states_ = std::make_shared<std::vector<StateType>>();
    goal_states_->reserve(100);
  }

 private:
  StateType DoSample(std::mt19937_64* generator, int thread_number) final {
    const auto uniform_unit_real_fn =
        [generator]() { return DrawUniformUnitReal(generator); };

    const auto [goal_states, goal_states_size] = GetViewOfGoalStates();
    if (uniform_unit_real_fn() > p_goal_sample_is_new_ &&
        goal_states_size > 0) {
      return goal_states->operator[](GetUniformRandomIndex(
          uniform_unit_real_fn, goal_states_size));
    } else {
      const StateType goal_sample =
          goal_sampler_->Sample(generator, thread_number);
      AddGoalState(goal_sample);
      return goal_sample;
    }
  }

  bool DoCheckGoalReached(const StateType& candidate) const final {
    const auto [goal_states, goal_states_size] = GetViewOfGoalStates();
    for (size_t index = 0; index < goal_states_size; ++index) {
      const StateType& goal = goal_states->operator[](index);
      if (this->planning_space().StateDistanceForwards(candidate, goal) <=
              this->goal_tolerance()) {
        return true;
      }
    }
    return false;
  }

  void AddGoalState(const StateType& goal_state) {
    // All goal addition operations must be serialized. Within addition, only
    // modifications to goal_states_ must be serialized with read operations.
    std::lock_guard<std::mutex> addition_lock(addition_mutex_);

    if (goal_states_->size() < goal_states_->capacity()) {
      // If adding the node will not reallocate the storage in goal_states_,
      // just add the new node directly.
      std::lock_guard<std::mutex> goal_states_lock(goal_states_mutex_);
      goal_states_->emplace_back(goal_state);
    } else {
      // If adding the node will cause a reallocation of the storage in
      // goal_states_, make a copy with the larger size, add to the copy, then
      // switch goal_states_.
      auto new_goal_states = std::make_shared<std::vector<StateType>>();

      // Reserve the same amount of space the reallocated vector would have.
      // Growing by a factor of 1.5 is better than growing by 2 as it allows
      // for more memory reuse.
      new_goal_states->reserve(goal_states_->size() + goal_states_->size() / 2);

      // Copy the goal states over.
      new_goal_states->insert(
          new_goal_states->end(), goal_states_->begin(), goal_states_->end());

      // Add the new goal state.
      new_goal_states->emplace_back(goal_state);

      // Switch to the new goal states.
      std::lock_guard<std::mutex> goal_states_lock(goal_states_mutex_);
      goal_states_ = new_goal_states;
    }
  }

  std::pair<std::shared_ptr<std::vector<StateType>>, size_t>
  GetViewOfGoalStates() const {
    std::lock_guard<std::mutex> nodes_lock(goal_states_mutex_);
    return std::make_pair(goal_states_, goal_states_->size());
  }

  mutable std::mutex goal_states_mutex_;
  mutable std::mutex addition_mutex_;
  const GoalSampler<StateType>* const goal_sampler_;
  const double p_goal_sample_is_new_;
  std::shared_ptr<std::vector<StateType>> goal_states_;
};

template <typename PlannerParamsType>
void ValidateBiRRTTerminationConditions(
    const PlannerParamsType& parameters, std::string_view planner_name) {
  if (parameters.time_limit.has_value() &&
      parameters.tree_growth_limit.has_value()) {
    DRAKE_THROW_UNLESS(parameters.time_limit.value() > 0.0);
    // There are always at least two nodes, one in each tree.
    DRAKE_THROW_UNLESS(parameters.tree_growth_limit.value() > 2);

    drake::log()->log(
        parameters.planner_log_level,
        "{} termination with time_limit {} and tree_growth_limit {}",
        planner_name, *parameters.time_limit, *parameters.tree_growth_limit);
  } else if (parameters.time_limit.has_value()) {
    DRAKE_THROW_UNLESS(parameters.time_limit.value() > 0.0);

    drake::log()->log(
        parameters.planner_log_level,
        "{} termination with time_limit {}",
        planner_name, *parameters.time_limit);
  } else if (parameters.tree_growth_limit.has_value()) {
    // There are always at least two nodes, one in each tree.
    DRAKE_THROW_UNLESS(parameters.tree_growth_limit.value() > 2);

    drake::log()->log(
        parameters.planner_log_level,
        "{} termination with tree_growth_limit {}",
        planner_name, *parameters.tree_growth_limit);
  } else {
    throw std::runtime_error(
        "parameters.time_limit and/or parameters.tree_growth_limit "
        "must be provided");
  }
}

template <typename PlannerParamsType>
void ValidateRRTTerminationConditions(
    const PlannerParamsType& parameters, std::string_view planner_name) {
  if (parameters.time_limit.has_value() &&
      parameters.tree_growth_limit.has_value()) {
    DRAKE_THROW_UNLESS(parameters.time_limit.value() > 0.0);
    // There is always at least one starting node.
    DRAKE_THROW_UNLESS(parameters.tree_growth_limit.value() > 1);

    drake::log()->log(
        parameters.planner_log_level,
        "{} termination with time_limit {} and tree_growth_limit {}",
        planner_name, *parameters.time_limit, *parameters.tree_growth_limit);
  } else if (parameters.time_limit.has_value()) {
    DRAKE_THROW_UNLESS(parameters.time_limit.value() > 0.0);

    drake::log()->log(
        parameters.planner_log_level,
        "{} termination with time_limit {}",
        planner_name, *parameters.time_limit);
  } else if (parameters.tree_growth_limit.has_value()) {
    // There is always at least one starting node.
    DRAKE_THROW_UNLESS(parameters.tree_growth_limit.value() > 1);

    drake::log()->log(
        parameters.planner_log_level,
        "{} termination with tree_growth_limit {}",
        planner_name, *parameters.tree_growth_limit);
  } else {
    throw std::runtime_error(
        "parameters.time_limit and/or parameters.tree_growth_limit "
        "must be provided");
  }
}

class SerialTerminationHelper {
 public:
  SerialTerminationHelper(
      const std::optional<double>& time_limit,
      const std::optional<int>& tree_growth_limit)
      : timeout_helper_(
            time_limit.value_or(std::numeric_limits<double>::max())),
        tree_growth_limit_(
            tree_growth_limit.value_or(std::numeric_limits<int>::max())) {}

  bool CheckRRTTermination(int64_t tree_size) {
    return CheckTermination(tree_size);
  }

  bool CheckBiRRTTermination(int64_t start_tree_size, int64_t goal_tree_size) {
    return CheckTermination(start_tree_size + goal_tree_size);
  }

 private:
  bool CheckTermination(int64_t tree_growth) {
    const bool time_exceeded = timeout_helper_.CheckOrStart();
    const bool tree_growth_exceeded = tree_growth > tree_growth_limit_;
    return time_exceeded || tree_growth_exceeded;
  }

  TimeoutCheckHelper timeout_helper_;
  const int tree_growth_limit_;
};

class ParallelTerminationHelper {
 public:
  ParallelTerminationHelper(
      const std::optional<double>& time_limit,
      const std::optional<int>& tree_growth_limit,
      const std::atomic<bool>* solution_found)
      : timeout_helper_(
            time_limit.value_or(std::numeric_limits<double>::max())),
        tree_growth_limit_(
            tree_growth_limit.value_or(std::numeric_limits<int>::max())),
        solution_found_(solution_found) {
    DRAKE_THROW_UNLESS(solution_found_ != nullptr);
  }

  bool CheckRRTTermination(int64_t tree_size) {
    return CheckTermination(tree_size);
  }

  bool CheckBiRRTTermination(int64_t start_tree_size, int64_t goal_tree_size) {
    return CheckTermination(start_tree_size + goal_tree_size);
  }

 private:
  bool CheckTermination(int64_t tree_growth) {
    const bool time_exceeded = timeout_helper_.CheckOrStart();
    const bool tree_growth_exceeded = tree_growth > tree_growth_limit_;
    return time_exceeded || tree_growth_exceeded || solution_found_->load();
  }

  TimeoutCheckHelper timeout_helper_;
  const int tree_growth_limit_;
  const std::atomic<bool>* const solution_found_;
};

inline PathPlanningErrors GetPlanningErrors(
    const std::optional<double>& time_limit, double planning_time,
    const std::optional<int>& tree_growth_limit, int total_states) {
  PathPlanningErrors errors;

  if (time_limit.has_value() && planning_time > time_limit.value()) {
    errors.insert(PathPlanningError::kTimeExceeded);
  }

  if (tree_growth_limit.has_value() &&
      total_states > tree_growth_limit.value()) {
    errors.insert(PathPlanningError::kTreeGrowthExceeded);
  }

  DRAKE_DEMAND(!errors.empty());

  return errors;
}

}  // namespace internal
}  // namespace planning
}  // namespace anzu
