#include "planning/goal_checker.h"

namespace anzu {
namespace planning {

template <typename StateType>
GoalChecker<StateType>::~GoalChecker() = default;

template <typename StateType>
GoalChecker<StateType>::GoalChecker() = default;

template <typename StateType>
GoalChecker<StateType>::GoalChecker(const GoalChecker& other) = default;

}  // namespace planning
}  // namespace anzu

ANZU_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_PLANNING_STATE_TYPES(
    class ::anzu::planning::GoalChecker)
