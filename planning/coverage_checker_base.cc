#include "drake/planning/coverage_checker_base.h"

namespace drake {
namespace planning {
using geometry::optimization::ConvexSet;

bool CoverageCheckerBase::CheckCoverage(
    const std::vector<std::unique_ptr<ConvexSet>>& current_sets) const {
  return DoCheckCoverage(current_sets);
}
}  // namespace planning
}  // namespace drake