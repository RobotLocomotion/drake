#include "planning/planning_space.h"

#include <common_robotics_utilities/openmp_helpers.hpp>

#include "drake/common/drake_throw.h"
#include "drake/common/text_logging.h"

namespace anzu {
namespace planning {
using common_robotics_utilities::openmp_helpers::GetContextOmpThreadNum;

template<typename StateType>
PlanningSpace<StateType>::~PlanningSpace() = default;

template<typename StateType>
double PlanningSpace<StateType>::CalcPathLength(
    const std::vector<StateType>& path) const {
  double path_length = 0.0;
  for (size_t index = 1; index < path.size(); ++index) {
    path_length += StateDistanceForwards(path.at(index - 1), path.at(index));
  }
  return path_length;
}

template<typename StateType>
ValidStarts<StateType> PlanningSpace<StateType>::ExtractValidStarts(
    const std::vector<StateType>& starts,
    const std::optional<int> thread_number) const {
  const int resolved_thread_number = ResolveThreadNumber(thread_number);

  std::vector<StateType> valid_starts;
  for (size_t start_index = 0; start_index < starts.size(); ++start_index) {
    const StateType& start = starts.at(start_index);
    if (CheckStateValidity(start, resolved_thread_number)) {
      valid_starts.push_back(start);
    } else {
      drake::log()->warn(
          "Start {}/{} is invalid", start_index + 1, starts.size());
    }
  }

  return ValidStarts<StateType>(valid_starts);
}

template<typename StateType>
ValidStartsAndGoals<StateType>
PlanningSpace<StateType>::ExtractValidStartsAndGoals(
    const std::vector<StateType>& starts,
    const std::vector<StateType>& goals,
    const std::optional<int> thread_number) const {
  const int resolved_thread_number = ResolveThreadNumber(thread_number);

  std::vector<StateType> valid_starts;
  for (size_t start_index = 0; start_index < starts.size(); ++start_index) {
    const StateType& start = starts.at(start_index);
    if (CheckStateValidity(start, resolved_thread_number)) {
      valid_starts.push_back(start);
    } else {
      drake::log()->warn(
          "Start {}/{} is invalid", start_index + 1, starts.size());
    }
  }

  std::vector<StateType> valid_goals;
  for (size_t goal_index = 0; goal_index < goals.size(); ++goal_index) {
    const StateType& goal = goals.at(goal_index);
    if (CheckStateValidity(goal, resolved_thread_number)) {
      valid_goals.push_back(goal);
    } else {
      drake::log()->warn(
          "Goal {}/{} is invalid", goal_index + 1, goals.size());
    }
  }

  return ValidStartsAndGoals<StateType>(valid_starts, valid_goals);
}

template<typename StateType>
bool PlanningSpace<StateType>::DoCheckPathValidity(
    const std::vector<StateType>& path, const int thread_number) const {
  if (path.size() > 1) {
    for (size_t index = 1; index < path.size(); ++index) {
      const StateType& previous = path.at(index - 1);
      const StateType& current = path.at(index);
      if (!CheckEdgeValidity(previous, current, thread_number)) {
        drake::log()->warn(
            "Edge from waypoint {} to waypoint {} invalid", index - 1, index);
        return false;
      }
    }
    return true;
  } else if (path.size() == 1) {
    return CheckStateValidity(path.at(0), thread_number);
  } else {
    throw std::runtime_error("Cannot check zero-waypoint paths for validity");
  }
}

template<typename StateType>
std::optional<StateType> PlanningSpace<StateType>::DoMaybeSampleValidState(
    const int max_attempts, const int thread_number) {
  for (int attempt = 0; attempt < max_attempts; ++attempt) {
    StateType sample = SampleState(thread_number);
    if (CheckStateValidity(sample, thread_number)) {
      return sample;
    }
  }
  return std::nullopt;
}

template<typename StateType>
StateType PlanningSpace<StateType>::SampleValidState(
    const int max_attempts, const std::optional<int> thread_number) {
  std::optional<StateType> maybe_valid_sample =
      MaybeSampleValidState(max_attempts, thread_number);
  if (maybe_valid_sample) {
    return maybe_valid_sample.value();
  } else {
    throw std::runtime_error(fmt::format(
        "Failed to sample valid state in {} attempts", max_attempts));
  }
}

template<typename StateType>
int PlanningSpace<StateType>::ResolveThreadNumber(
    const std::optional<int> thread_number) {
  const int resolved_thread_number =
      thread_number.has_value() ? *thread_number : GetContextOmpThreadNum();
  return resolved_thread_number;
}

template<typename StateType>
PlanningSpace<StateType>::PlanningSpace(
    const PlanningSpace<StateType>& other) = default;

template<typename StateType>
PlanningSpace<StateType>::PlanningSpace(
    const uint64_t seed, const Parallelism parallelism, const bool is_symmetric)
    : random_source_(seed, parallelism), parallelism_(parallelism),
      is_symmetric_(is_symmetric) {}

}  // namespace planning
}  // namespace anzu

ANZU_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_PLANNING_STATE_TYPES(
    class ::anzu::planning::PlanningSpace)
