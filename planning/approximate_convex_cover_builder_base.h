#pragma once

#include <vector>

#include "drake/geometry/optimization/convex_set.h"
#include "drake/planning/collision_checker.h"

namespace drake {
namespace planning {

/**
 * An abstract base class for implementing methods for checking whether a set of
 * convex sets provides sufficient coverage for solving approximate convex
 * cover.
 */
class CoverageCheckerBase {
 public:
  /**
   * Check that the current sets provide sufficient coverage.
   */
  virtual bool CheckCoverage(
      const std::vector<geometry::optimization::ConvexSet>& current_sets)
      const = 0;
};

/**
 * An abstract base class for implementing ways to sample points from the
 * underlying space an approximate convex cover builder wishes to cover.
 */
class SpaceSamplerBase {
 public:
  /**
   * Sample num_points from the underlying space and return these points as a
   * matrix where each column represents an underlying point.
   */
  virtual Eigen::MatrixXd SamplePoints(const int num_points) const = 0;
};

/**
 *
 * @param checker
 * @param points
 * @param options
 * @param minimum_clique_size
 * @param parallelize
 * @return
 */

std::vector<geometry::optimization::HPolyhedron> IrisRegionsFromCliqueCover(
    const CollisionChecker& checker,
    const Eigen::Ref<const Eigen::MatrixXd>& points,
    const geometry::optimization::IrisOptions& options =
        geometry::optimization::IrisOptions(),
    int minimum_clique_size = 3, bool parallelize = true);

}  // namespace planning
}  // namespace drake