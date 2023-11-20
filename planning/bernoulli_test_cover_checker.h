#pragma once

#include <memory>
#include <vector>

#include "drake/common/copyable_unique_ptr.h"
#include "drake/planning/approximate_convex_cover_builder_base.h"

namespace drake {
namespace planning {

class BernoulliTestCoverChecker final : public CoverageCheckerBase {
 public:
  /**
   * @param alpha_threshold CheckCoverage returns True if 1-alpha fraction of
   * the sampled points are in the current sets
   * @param num_points_per_check Number of points to sample when checking
   * coverage.
   * @param num_threads Number of threads to use when checking coverage. Numbers
   * less than 1 will be interpretted as hardware concurrency.
   */
  BernoulliTestCoverChecker(const int alpha_threshold,
                            const int num_points_per_check,
                            std::unique_ptr<PointSamplerBase> point_sampler,
                            const int num_threads = -1);

  bool CheckCoverage(
      const std::vector<std::unique_ptr<ConvexSet>>& current_sets) const;

 private:
  const int alpha_threshold_;
  const int num_points_per_check_;
  const std::unique_ptr<PointSamplerBase> point_sampler_;
  const int num_threads_;
};

}  // namespace planning
}  // namespace drake
