#include "drake/planning/sampling_based/dev/path_planning_error.h"

#include "drake/common/drake_assert.h"

namespace drake {
namespace planning {

std::string_view to_string(const PathPlanningError& error) {
  // A macro to help avoid copy-paste mistakes and cross-check the max value.
#define DRAKE_ADD_CASE(name)                                                   \
  case PathPlanningError::name: {                                              \
    static_assert(PathPlanningError::name <= internal::kMaxPathPlanningError); \
    return #name;                                                              \
  }
  switch (error) {
    DRAKE_ADD_CASE(kUnknown)
    DRAKE_ADD_CASE(kNoValidStart)
    DRAKE_ADD_CASE(kNoValidGoal)
    DRAKE_ADD_CASE(kCannotConnectStart)
    DRAKE_ADD_CASE(kCannotConnectGoal)
    DRAKE_ADD_CASE(kCannotFindPath)
    DRAKE_ADD_CASE(kTimeExceeded)
    DRAKE_ADD_CASE(kTreeGrowthExceeded)
  }
#undef DRAKE_ADD_CASE
  DRAKE_UNREACHABLE();
}  // namespace planning

}  // namespace planning
}  // namespace drake
