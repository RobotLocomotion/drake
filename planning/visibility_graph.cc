#include "drake/planning/visibility_graph.h"

#include <iterator>
#include <vector>

#include <common_robotics_utilities/openmp_helpers.hpp>
#if defined(_OPENMP)
#include <omp.h>
#endif

namespace drake {
namespace planning {

using common_robotics_utilities::openmp_helpers::DegreeOfParallelism;

namespace {

/* A custom iterator to turn std::vector<std::vector<int>> edges into an
iterator over Triplets for Eigen::SparseMatrix<bool>. The edges data structure
only contains the lower-triangular entries, but edges are undirected, so the
resulting SparseMatrix<bool> needs to be symmetric. */
class EdgesIterator
    : public std::iterator<std::input_iterator_tag, Eigen::Triplet<bool>> {
 public:
  EdgesIterator(const std::vector<std::vector<int>>& edges, int outer_idx = 0,
                int inner_idx = 0, bool upper_triangle = false)
      : edges_(edges),
        outer_idx_(outer_idx),
        inner_idx_(inner_idx),
        upper_triangle_(upper_triangle) {
    skip_empty_outer();
  }

  EdgesIterator begin() { return EdgesIterator(edges_, 0, 0, false); }
  EdgesIterator end() { return EdgesIterator(edges_, edges_.size(), 0, false); }

  // Returns a Triplet* with the address of a static Eigen::Triplet<bool> object
  // representing the current edge. This works in this context because
  // setFromTriplets() just reads from the iterators and doesn't store them for
  // later.
  const Eigen::Triplet<bool>* operator*() const {
    static Eigen::Triplet<bool> e;
    if (upper_triangle_) {
      e = Eigen::Triplet<bool>(edges_[outer_idx_][inner_idx_], outer_idx_,
                               true);
    } else {
      e = Eigen::Triplet<bool>(outer_idx_, edges_[outer_idx_][inner_idx_],
                               true);
    }
    return &e;
  }

  const Eigen::Triplet<bool>* operator->() const { return **this; }

  // Advance the iterator to the next edge.
  EdgesIterator& operator++() {
    upper_triangle_ = !upper_triangle_;
    if (!upper_triangle_) {
      if (++inner_idx_ >= ssize(edges_[outer_idx_])) {
        ++outer_idx_;
        inner_idx_ = 0;
      }
      skip_empty_outer();
    }
    return *this;
  }

  // Compares two iterators.
  bool operator!=(const EdgesIterator& other) const {
    return outer_idx_ != other.outer_idx_ || inner_idx_ != other.inner_idx_ ||
           upper_triangle_ != other.upper_triangle_;
  }

 private:
  // If a row is empty, skip to the next non-empty row
  void skip_empty_outer() {
    while (outer_idx_ < ssize(edges_) && edges_[outer_idx_].empty()) {
      ++outer_idx_;
    }
  }

  // Iterators pointing to each vector in the list of vectors.
  const std::vector<std::vector<int>>& edges_;
  int outer_idx_;
  int inner_idx_;
  bool upper_triangle_;
};

}  // namespace

Eigen::SparseMatrix<bool> VisibilityGraph(
    const CollisionChecker& checker,
    const Eigen::Ref<const Eigen::MatrixXd>& points, bool parallelize) {
  DRAKE_THROW_UNLESS(checker.plant().num_positions() == points.rows());

  const int num_points = points.cols();
  // Choose std::vector as a thread-safe data structure for the parallel
  // evaluations.
  std::vector<std::vector<int>> edges(num_points);

  const DegreeOfParallelism parallelism(
      checker.GetNumberOfThreads(parallelize));

  CRU_OMP_PARALLEL_FOR_DEGREE(parallelism)
  for (int i = 1; i < num_points; ++i) {
    for (int j = 0; j < i; ++j) {
      if (checker.CheckEdgeCollisionFree(points.col(i), points.col(j))) {
        edges[i].emplace_back(j);
        log()->info("Adding edge between {} and {}", i, j);
      }
    }
  }

  // Convert edges into the SparseMatrix format, using a custom iterator to
  // avoid explicitly copying the data into a list of Eigen::Triplet.
  Eigen::SparseMatrix<bool> mat(num_points, num_points);
  EdgesIterator edges_iterator(edges);
  mat.setFromTriplets(edges_iterator.begin(), edges_iterator.end());
  return mat;
}

}  // namespace planning
}  // namespace drake
