#pragma once

#include <memory>

#include "drake/common/drake_copyable.h"
#include "drake/geometry/optimization/convex_set.h"

namespace drake {
namespace planning {

using geometry::optimization::ConvexSet;

/**
 * An abstract base class for implementing ways to build a convex set from a
 * clique of points
 */
class ConvexSetFromCliqueBuilderBase {
 public:
  ConvexSetFromCliqueBuilderBase() = default;
  /**
   * Given a set of points, build a convex set.
   */
  copyable_unique_ptr<ConvexSet> BuildConvexSet(
      const Eigen::Ref<const Eigen::MatrixXd>& clique_points);

  virtual ~ConvexSetFromCliqueBuilderBase() {}

 protected:
  // We put the copy/move/assignment constructors as protected to avoid copy
  // slicing. The inherited final subclasses should put them in public
  // functions.
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(ConvexSetFromCliqueBuilderBase);

  virtual copyable_unique_ptr<ConvexSet> DoBuildConvexSet(
      const Eigen::Ref<const Eigen::MatrixXd>& clique_points) = 0;
};

}  // namespace planning
}  // namespace drake
