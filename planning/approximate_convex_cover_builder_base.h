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

struct ApproximateConvexCoverFromCliqueCoverOptions {
  int num_sampled_points{100};
  int minimum_clique_size{3};
};

void ApproximateConvexCoverFromCliqueCover(
    CoverageCheckerBase* coverage_checker, PointSamplerBase* point_sampler,
    AdjacencyMatrixBuilderBase* adjacency_matrix_builder,
    MaxCliqueSolverBase* max_clique_solver,
    const std::vector<std::unique_ptr<ConvexSetFromCliqueBuilderBase>>&
        set_builders,
    const ApproximateConvexCoverFromCliqueCoverOptions& options,
    ConvexSets* convex_sets);

}  // namespace planning
}  // namespace drake
