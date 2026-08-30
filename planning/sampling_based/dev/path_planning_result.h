#pragma once

#include <limits>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/planning/sampling_based/dev/default_state_types.h"
#include "drake/planning/sampling_based/dev/path_planning_errors.h"

namespace drake {
namespace planning {

/** Holds the results from planning {path, length, errors} where `path` is the
planned sequence of states, `length` is the length of the planned path,
and `errors` contains any errors. If a solution cannot be found, `path` is
empty, `length` is infinity, and `errors` is non-empty. */
template <typename StateType>
class PathPlanningResult {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(PathPlanningResult);

  /** Constructs a PathPlanningResult with the specified `path`. */
  PathPlanningResult(std::vector<StateType> path, double path_length);

  /** Constructs a PathPlanningResult with the specified errors.
  @pre errors is not empty. */
  explicit PathPlanningResult(PathPlanningErrors errors);

  /** Constructs a PathPlanningResult with the specified error. */
  explicit PathPlanningResult(PathPlanningError error);

  /** Default-constructs a PathPlanningResult that represents an error. */
  PathPlanningResult();

  ~PathPlanningResult();

  const std::vector<StateType>& path() const { return path_; }

  double path_length() const { return path_length_; }

  const PathPlanningErrors& errors() const { return errors_; }

  bool has_solution() const { return errors().empty(); }

 private:
  std::vector<StateType> path_;
  double path_length_{std::numeric_limits<double>::infinity()};
  PathPlanningErrors errors_{PathPlanningError::kUnknown};
};

}  // namespace planning
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_PLANNING_STATE_TYPES(
    class ::drake::planning::PathPlanningResult)
