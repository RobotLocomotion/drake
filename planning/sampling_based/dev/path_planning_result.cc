#include "planning/path_planning_result.h"

#include <utility>

namespace anzu {
namespace planning {

template <typename StateType>
PathPlanningResult<StateType>::PathPlanningResult(std::vector<StateType> path,
                                                  const double path_length)
    : path_(std::move(path)), path_length_(path_length), errors_() {
  DRAKE_THROW_UNLESS(this->path().size() > 0);
  DRAKE_THROW_UNLESS(std::isfinite(this->path_length()));
}

template <typename StateType>
PathPlanningResult<StateType>::PathPlanningResult(PathPlanningErrors errors)
    : errors_(std::move(errors)) {
  DRAKE_THROW_UNLESS(!this->errors().empty());
}

template <typename StateType>
PathPlanningResult<StateType>::PathPlanningResult(
    const PathPlanningError error)
    : PathPlanningResult(PathPlanningErrors(error)) {}

template <typename StateType>
PathPlanningResult<StateType>::PathPlanningResult() = default;

template <typename StateType>
PathPlanningResult<StateType>::~PathPlanningResult() = default;

}  // namespace planning
}  // namespace anzu

ANZU_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_PLANNING_STATE_TYPES(
    class ::anzu::planning::PathPlanningResult)
