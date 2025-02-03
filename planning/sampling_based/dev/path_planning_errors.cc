#include "planning/path_planning_errors.h"

namespace anzu {
namespace planning {

static_assert(sizeof(PathPlanningErrors) <= 8, "Should fit within 1 word");

std::vector<PathPlanningError> PathPlanningErrors::to_vector() const {
  std::vector<PathPlanningError> result;
  for (ErrorAsInt i = 0; i < N; ++i) {
    if (bits_.test(i)) {
      result.push_back(PathPlanningError{i});
    }
  }
  return result;
}

std::string to_string(const PathPlanningErrors& errors) {
  std::vector<std::string_view> error_strings;
  for (const PathPlanningError& error : errors.to_vector()) {
    error_strings.push_back(to_string(error));
  }
  return fmt::format("{{{}}}", fmt::join(error_strings, ", "));
}

}  // namespace planning
}  // namespace anzu
