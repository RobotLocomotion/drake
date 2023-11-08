#pragma once

#include <vector>

#include "drake/geometry/optimization/convex_set.h"
#include "drake/planning/graph_algorithms/max_clique.h"

namespace drake {
namespace planning {

using geometry::optimization::ConvexSet;
using graph_algorithms::MaxCliqueOptions;

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
      const std::vector<ConvexSet>& current_sets) const = 0;

  virtual ~CoverageCheckerBase() {}

 protected:
  // We put the copy/move/assignment constructors as protected to avoid copy
  // slicing. The inherited final subclasses should put them in public
  // functions.
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(CoverageCheckerBase);
};

/**
 * An abstract base class for implementing ways to sample points from the
 * underlying space an approximate convex cover builder wishes to cover.
 */
class PointSamplerBase {
 public:
  /**
   * Sample num_points from the underlying space and return these points as a
   * matrix where each column represents an underlying point.
   * @param num_threads the number of threads used to sample points.
   */
  virtual Eigen::MatrixXd SamplePoints(int num_points) const = 0;

  virtual ~PointSamplerBase() {}

 protected:
  // We put the copy/move/assignment constructors as protected to avoid copy
  // slicing. The inherited final subclasses should put them in public
  // functions.
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(PointSamplerBase);
};

class AdjacencyMatrixBuilderBase {
 public:
  /**
   * Given points in the space, build the adjacency matrix of these points
   * describing the desired graphical relationship.
   * @param points to be used as the vertices of the graph.
   */
  virtual Eigen::SparseMatrix<bool>& BuildAdjacencyMatrix(
      const Eigen::Ref<const Eigen::MatrixXd>& points) const = 0;

  virtual ~AdjacencyMatrixBuilderBase() {}

 protected:
  // We put the copy/move/assignment constructors as protected to avoid copy
  // slicing. The inherited final subclasses should put them in public
  // functions.
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(AdjacencyMatrixBuilderBase);
};

/**
 * An abstract base class for implementing ways to build a convex set from a
 * clique of points
 */
class ConvexSetFromCliqueBuilderBase {
 public:
  /**
   * Given a set of points, build a convex set.
   */
  virtual ConvexSet BuildConvexSet(
      const Eigen::Ref<const Eigen::MatrixXd>& clique) const = 0;

  virtual ~ConvexSetFromCliqueBuilderBase() {}

 protected:
  // We put the copy/move/assignment constructors as protected to avoid copy
  // slicing. The inherited final subclasses should put them in public
  // functions.
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(ConvexSetFromCliqueBuilderBase);
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

class ApproximateConvexCoverFromCliqueCoverOptions {
 public:
  ApproximateConvexCoverFromCliqueCoverOptions(
      const CoverageCheckerBase* coverage_checker,
      const PointSamplerBase* point_sampler,
      const AdjacencyMatrixBuilderBase* adjacency_matrix_builder,
      const ConvexSetFromCliqueBuilderBase* set_builder,
      const MaxCliqueOptions& max_clique_options, int num_sampled_points,
      int minimum_clique_size = 3, int num_threads = -1);

  [[nodiscard]] const CoverageCheckerBase* coverage_checker() const {
    return coverage_checker_;
  }

  [[nodiscard]] const PointSamplerBase* point_sampler() const {
    return point_sampler_;
  }

  [[nodiscard]] const AdjacencyMatrixBuilderBase* adjacency_matrix_builder()
      const {
    return adjacency_matrix_builder_;
  }

  [[nodiscard]] const ConvexSetFromCliqueBuilderBase* set_builder() const {
    return set_builder_;
  }

  [[nodiscard]] const MaxCliqueOptions max_clique_options() const {
    return max_clique_options_;
  }

  [[nodiscard]] int num_sampled_points() const { return num_sampled_points_; }

  [[nodiscard]] int minimum_clique_size() const { return minimum_clique_size_; }

  [[nodiscard]] int num_threads() const { return num_threads_; }

 private:
  const CoverageCheckerBase* coverage_checker_;

  const PointSamplerBase* point_sampler_;

  const AdjacencyMatrixBuilderBase* adjacency_matrix_builder_;

  const ConvexSetFromCliqueBuilderBase* set_builder_;

  const MaxCliqueOptions max_clique_options_;

  const int num_sampled_points_;

  const int minimum_clique_size_;

  const int num_threads_;
};

std::vector<ConvexSet> ApproximateConvexCoverFromCliqueCover(
    const ApproximateConvexCoverFromCliqueCoverOptions& options);

}  // namespace planning
}  // namespace drake