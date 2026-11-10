#pragma once

#include "drake/planning/sampling_based/dev/default_state_types.h"

namespace drake {
namespace planning {

/// Interface used by RRT and ParallelRRT planners to check if a candidate state
/// satisfies goal conditions. Since ParallelRRT performs goal checks in
/// parallel, implementations of GoalChecker must be thread safe.
template <typename StateType>
class GoalChecker {
 public:
  // The copy constructor is protected to allow derived classes to be copyable.
  // Does not allow copy, move, or assignment.
  GoalChecker(GoalChecker&&) = delete;
  GoalChecker<StateType>& operator=(const GoalChecker&) = delete;
  GoalChecker<StateType>& operator=(GoalChecker&&) = delete;

  virtual ~GoalChecker();

  /// Checks if the provided state `candidate` satisfies goal conditions. For
  /// thread safety when using a parallel planner, a thread number is provided
  /// for use with other resources, such as performing collision checks on a
  /// PlanningSpace (e.g. a goal check that the provided candidate has a
  /// collision-free edge to a specific state, or is sufficiently far from
  /// collision).
  bool CheckGoalReached(const StateType& candidate, int thread_number) const {
    return DoCheckGoalReached(candidate, thread_number);
  }

 protected:
  GoalChecker();

  GoalChecker(const GoalChecker& other);

  /// Derived goal checkers must implement goal checking.
  virtual bool DoCheckGoalReached(const StateType& candidate,
                                  int thread_number) const = 0;
};

}  // namespace planning
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_PLANNING_STATE_TYPES(
    class ::drake::planning::GoalChecker)
