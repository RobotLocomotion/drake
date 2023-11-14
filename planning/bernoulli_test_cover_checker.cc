#include "drake/planning/bernoulli_test_cover_checker.h"

#if defined(_OPENMP)
#include <omp.h>
#endif

#include <thread>

namespace drake {
namespace planning {

BernoulliTestCoverChecker::BernoulliTestCoverChecker(
    const int alpha_threshold, const int num_points_per_check,
    std::unique_ptr<PointSamplerBase> point_sampler,
    const int num_threads)
    : alpha_threshold_(alpha_threshold),
      num_points_per_check_(num_points_per_check),
      point_sampler_(std::move(point_sampler)),
      num_threads_{
          num_threads > 0
              ? num_threads
              : static_cast<int>(std::thread::hardware_concurrency())} {};

bool BernoulliTestCoverChecker::CheckCoverage(
      const std::vector<std::unique_ptr<geometry::optimization::ConvexSet>>& current_sets) const {
  Eigen::MatrixXd sampled_points =
      point_sampler_->SamplePoints(num_points_per_check_);
  std::atomic<double> num_in_sets{0};
#if defined(_OPENMP)
#pragma omp parallel for num_threads(num_threads_)
#endif
  for (int i = 0; i < sampled_points.cols(); ++i) {
    for (const auto& set : current_sets) {
      if (set->PointInSet(sampled_points.col(i))) {
        num_in_sets.fetch_add(1);
        break;
      }
    }
  }
  return num_in_sets/num_points_per_check_ >= 1-alpha_threshold_;
}
}  // namespace planning
}  // namespace drake