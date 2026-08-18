#include "drake/planning/sampling_based/dev/goal_checker.h"

namespace drake {
namespace planning {

template <typename StateType>
GoalChecker<StateType>::~GoalChecker() = default;

template <typename StateType>
GoalChecker<StateType>::GoalChecker() = default;

template <typename StateType>
GoalChecker<StateType>::GoalChecker(const GoalChecker& other) = default;

}  // namespace planning
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_PLANNING_STATE_TYPES(
    class ::drake::planning::GoalChecker)
