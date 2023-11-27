#include "drake/planning/coverage_checker_base.h"

namespace drake {
namespace planning {
using geometry::optimization::ConvexSets;

bool CoverageCheckerBase::CheckCoverage(
    const ConvexSets& current_sets) const {
  return DoCheckCoverage(current_sets);
}
}  // namespace planning
}  // namespace drake