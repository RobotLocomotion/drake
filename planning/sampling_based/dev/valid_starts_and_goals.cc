#include "planning/valid_starts_and_goals.h"

#include <utility>

#include "drake/common/text_logging.h"

namespace anzu {
namespace planning {

template <typename StateType>
ValidStartsAndGoals<StateType>::ValidStartsAndGoals(
    std::vector<StateType> valid_starts, std::vector<StateType> valid_goals)
    : valid_starts_(std::move(valid_starts)),
      valid_goals_(std::move(valid_goals)) {
  SetErrors();
}

template <typename StateType>
ValidStartsAndGoals<StateType>::ValidStartsAndGoals()
    : ValidStartsAndGoals<StateType>({}, {}) {}

template <typename StateType>
void ValidStartsAndGoals<StateType>::SetErrors() {
  PathPlanningErrors errors;
  if (valid_starts().empty()) {
    errors.insert(PathPlanningError::kNoValidStart);
  }
  if (valid_goals().empty()) {
    errors.insert(PathPlanningError::kNoValidGoal);
  }
  errors_ = errors;
}

}  // namespace planning
}  // namespace anzu

ANZU_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_PLANNING_STATE_TYPES(
    class ::anzu::planning::ValidStartsAndGoals)
