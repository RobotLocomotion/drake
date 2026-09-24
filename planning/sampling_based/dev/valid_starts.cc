#include "drake/planning/sampling_based/dev/valid_starts.h"

#include <utility>

namespace drake {
namespace planning {

template <typename StateType>
ValidStarts<StateType>::ValidStarts(std::vector<StateType> valid_starts)
    : valid_starts_(std::move(valid_starts)) {
  SetErrors();
}

template <typename StateType>
ValidStarts<StateType>::ValidStarts()
    : ValidStarts<StateType>(std::vector<StateType>{}) {}

template <typename StateType>
void ValidStarts<StateType>::SetErrors() {
  PathPlanningErrors errors;
  if (valid_starts().empty()) {
    errors.insert(PathPlanningError::kNoValidStart);
  }
  errors_ = errors;
}

}  // namespace planning
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_PLANNING_STATE_TYPES(
    class ::drake::planning::ValidStarts)
