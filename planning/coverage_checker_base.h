#pragma once

#include <memory>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/geometry/optimization/convex_set.h"

namespace drake {
namespace planning {
using geometry::optimization::ConvexSets;

/**
 * An interface for implementing methods for checking whether a set of
 * convex sets provides sufficient coverage of an underlying space. This
 * interface is used in ApproximateConvexCoverBuilder.
 */
class CoverageCheckerBase {
 public:
  CoverageCheckerBase() = default;
  /**
   * Check that the current sets provide sufficient coverage.
   */
  //  virtual bool CheckCoverage(
  //      const std::vector<ConvexSet>& current_sets) const = 0;
  //
  //  /**
  //   * Check that the current sets provide sufficient coverage.
  //   */
  //  virtual bool CheckCoverage(
  //      const std::queue<ConvexSet>& current_sets) const = 0;

  //  virtual bool CheckCoverage(
  //      const std::queue<std::unique_ptr<ConvexSet>>& current_sets) const = 0;

  // TODO(Alexandre.Amice) Coverage checker doesn't necessarily need unique pointers to convex sets. Consider making a template version.
  bool CheckCoverage(const ConvexSets& current_sets) const;

  virtual ~CoverageCheckerBase() {}

 protected:
  // We put the copy/move/assignment constructors as protected to avoid copy
  // slicing. The inherited final subclasses should put them in public
  // functions.
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(CoverageCheckerBase);

  virtual bool DoCheckCoverage(const ConvexSets& current_sets) const = 0;
};

}  // namespace planning
}  // namespace drake
