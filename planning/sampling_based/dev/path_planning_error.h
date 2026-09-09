#pragma once

#include <cstdint>
#include <string_view>

#include "drake/common/fmt.h"

namespace drake {
namespace planning {

/** Possible error codes from path planning. Typically, error codes will be
communicated as part of a PathPlanningResult. */
enum class PathPlanningError : uint8_t {
  /** Some non-specific error; this is the default value. */
  kUnknown,

  /** No valid starting state was provided (e.g., no starting state provided, or
  all starting states were identified as invalid by the planning space in use).
  */
  kNoValidStart,

  /** No valid goal state was provided (e.g., no goal state provided, or all
  goal states were identified as invalid by the planning space in use). */
  kNoValidGoal,

  /** Provided starting state(s) were valid, but could not be connected to the
  rest of the planning problem (e.g., starting state(s) could not be connected
  to the PRM roadmap in use). */
  kCannotConnectStart,

  /** Provided goal state(s) were valid, but could not be connected to the rest
  of the planning problem (e.g., goal state(s) could not be connected to the PRM
  roadmap in use). */
  kCannotConnectGoal,

  /** A path between starting state(s) and goal state(s) could not be found. For
  complete planners, like A* or PRM queries, this means that the problem as
  posed does not have a solution (e.g., a different discretization or larger
  roadmap may be necessary). */
  kCannotFindPath,

  /** Planner exceeded time limit while attempting to find a path between
  starting state(s) and goal state(s). For (Bi)RRT planners, the problem may
  still have a solution, but it will require additional time to solve. */
  kTimeExceeded,

  /** Planner exceeded tree growth limit while attempting to find a path between
  starting state(s) and goal state(s). For (Bi)RRT planners, the problem may
  still have a solution, but it will require additional tree growth to solve. */
  kTreeGrowthExceeded,
};

/** Returns a string representation of `error` for debugging output. */
std::string_view to_string(const PathPlanningError& error);

namespace internal {
// This should always be the largest item of the enum. (Our *.cc file enforces
// this via a static_assert.)
constexpr PathPlanningError kMaxPathPlanningError =
    PathPlanningError::kTreeGrowthExceeded;
}  // namespace internal

}  // namespace planning
}  // namespace drake

DRAKE_FORMATTER_AS(, drake::planning, PathPlanningError, x,
                   drake::planning::to_string(x))
