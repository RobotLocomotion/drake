#include "drake/planning/approximate_convex_cover_builder_base.h"

#include <thread>

#if defined(_OPENMP)
#include <omp.h>
#endif

namespace drake {
namespace planning {

ApproximateConvexCoverFromCliqueCoverOptions::
    ApproximateConvexCoverFromCliqueCoverOptions(
        const CoverageCheckerBase* coverage_checker,
        const PointSamplerBase* point_sampler,
        const ConvexSetFromCliqueBuilderBase* set_builder,
        const MaxCliqueOptions& max_clique_options, int num_sampled_points,
        int minimum_clique_size, int num_threads)
    : coverage_checker_{coverage_checker},
      point_sampler_{point_sampler},
      set_builder_{set_builder},
      max_clique_options_{max_clique_options},
      num_sampled_points_{num_sampled_points},
      minimum_clique_size_{minimum_clique_size},
      num_threads_{num_threads} {};

std::vector<ConvexSet> ApproximateConvexCoverFromCliqueCover(
    const ApproximateConvexCoverFromCliqueCoverOptions& options) {
  const int num_threads =
      options.num_threads() < 1
          ? static_cast<int>(std::thread::hardware_concurrency())
          : options.num_threads();
  bool parallelize = options.num_threads() == 1;

  std::vector<std::thread>
  std::vector<ConvexSet> ret;
  while (!options.coverage_checker()->CheckCoverage(ret)) {
    const Eigen::MatrixXd points =
        options.point_sampler()->SamplePoints(options.num_sampled_points());
    const Eigen::SparseMatrix<bool> adjacency_matrix =
        options.adjacency_matrix_builder()->BuildAdjacencyMatrix(points);
    std::thread clique_cover_thread{};
  }
}

}  // namespace planning
}  // namespace drake
