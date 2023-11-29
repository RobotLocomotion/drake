#pragma once

#include <memory>
#include <queue>
#include <vector>

#include "drake/geometry/optimization/convex_set.h"
#include "drake/planning/adjacency_matrix_builder_base.h"
#include "drake/planning/convex_set_from_clique_builder_base.h"
#include "drake/planning/coverage_checker_base.h"
#include "drake/planning/graph_algorithms/max_clique_solver_base.h"
#include "drake/planning/point_sampler_base.h"

namespace drake {
namespace planning {

using geometry::optimization::ConvexSet;
using geometry::optimization::ConvexSets;
using graph_algorithms::MaxCliqueSolverBase;

/**
 * TODO Fill in this constructor
 *
 * Throw if set_builder is empty.
 */
struct ApproximateConvexCoverFromCliqueCoverOptions {
 public:
  //  ApproximateConvexCoverFromCliqueCoverOptions() = default;
  //  ApproximateConvexCoverFromCliqueCoverOptions(
  //      std::unique_ptr<CoverageCheckerBase> coverage_checker,
  //      std::unique_ptr<PointSamplerBase> point_sampler,
  //      std::unique_ptr<AdjacencyMatrixBuilderBase> adjacency_matrix_builder,
  //      std::unique_ptr<MaxCliqueSolverBase> max_clique_solver,
  //      std::vector<std::unique_ptr<ConvexSetFromCliqueBuilderBase>>
  //      set_builder, int num_sampled_points, int minimum_clique_size = 3);
  //
  //  [[nodiscard]] CoverageCheckerBase* coverage_checker() const {
  //    return coverage_checker_.get();
  //  }
  //
  //  [[nodiscard]] PointSamplerBase* point_sampler() const {
  //    return point_sampler_.get();
  //  }
  //
  //  [[nodiscard]] AdjacencyMatrixBuilderBase* adjacency_matrix_builder() const
  //  {
  //    return adjacency_matrix_builder_.get();
  //  }
  //
  //  [[nodiscard]] MaxCliqueSolverBase* max_clique_solver() const {
  //    return max_clique_solver_.get();
  //  }
  //
  //  [[nodiscard]] int num_set_builders() const { return set_builders_.size();
  //  }
  //
  //  [[nodiscard]] ConvexSetFromCliqueBuilderBase* get_set_builder(int i) const
  //  {
  //    return set_builders_.at(i).get();
  //  }
  //
  //  [[nodiscard]] int num_sampled_points() const { return num_sampled_points_;
  //  }
  //
  //  [[nodiscard]] int minimum_clique_size() const { return
  //  minimum_clique_size_; }

  // private:
  //  std::unique_ptr<CoverageCheckerBase> coverage_checker_;
  //
  //  std::unique_ptr<PointSamplerBase> point_sampler_;
  //
  //  std::unique_ptr<AdjacencyMatrixBuilderBase> adjacency_matrix_builder_;
  //
  //  std::unique_ptr<MaxCliqueSolverBase> max_clique_solver_;
  //
  //  std::vector<std::unique_ptr<ConvexSetFromCliqueBuilderBase>>
  //  set_builders_;

  int num_sampled_points{100};

  int minimum_clique_size{3};
};

// void ApproximateConvexCoverFromCliqueCover(
//    const ApproximateConvexCoverFromCliqueCoverOptions& options,
//    ConvexSets* convex_sets);

void ApproximateConvexCoverFromCliqueCover(
    CoverageCheckerBase* coverage_checker,
    PointSamplerBase* point_sampler,
    AdjacencyMatrixBuilderBase* adjacency_matrix_builder,
    MaxCliqueSolverBase* max_clique_solver,
    const std::vector<std::unique_ptr<ConvexSetFromCliqueBuilderBase>>& set_builders,
    const ApproximateConvexCoverFromCliqueCoverOptions& options,
    ConvexSets* convex_sets);

}  // namespace planning
}  // namespace drake