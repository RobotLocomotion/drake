#include "drake/planning/visibility_graph.h"

#include <iterator>
#include <vector>

#include <common_robotics_utilities/openmp_helpers.hpp>

#if defined(_OPENMP)
#include <omp.h>
#endif

using common_robotics_utilities::openmp_helpers::GetContextOmpThreadNum;
using common_robotics_utilities::openmp_helpers::IsOmpEnabledInBuild;

namespace drake {
namespace planning {
namespace {

/* A custom iterator that turns undirected edges encoded in the given
std::vector<std::vector<int>> `edges` into a sequence of Eigen::SparseMatrix
triplets which define a symmetric matrix. The edges in `edges` are undirected,
so each such edge creates two triplets: (i, j, value) and (j, i, value).
*/
class EdgesIterator {
 public:
  using T = Eigen::Triplet<bool>;
  using iterator_category = std::input_iterator_tag;
  using value_type = T;
  using difference_type = std::ptrdiff_t;
  using pointer = T*;
  using reference = T&;

  EdgesIterator(const std::vector<std::vector<int>>& edges, int outer_idx = 0,
                int inner_idx = 0, bool upper_triangle = false)
      : edges_(edges),
        outer_idx_(outer_idx),
        inner_idx_(inner_idx),
        upper_triangle_(upper_triangle) {
    skip_empty_outer();
  }

  EdgesIterator begin() {
    return EdgesIterator(edges_, /* outer_idx = */ 0, /* inner_idx = */ 0,
                         /* upper_triangle = */ false);
  }

  // To implement end(), we create the element that will be obtained by
  // incrementing past the final reasonable edge output.
  EdgesIterator end() {
    return EdgesIterator(edges_, /* outer_idx = */ edges_.size(),
                         /* inner_idx = */ 0, /* upper_triangle = */ false);
  }

  // Returns a Triplet* with the address of a static Eigen::Triplet<bool> object
  // representing the current edge. This works in this context because
  // setFromTriplets() just reads from the iterators and doesn't store them for
  // later.
  const Eigen::Triplet<bool>& operator*() const {
    static Eigen::Triplet<bool> e;
    if (upper_triangle_) {
      e = Eigen::Triplet<bool>(edges_[outer_idx_][inner_idx_], outer_idx_,
                               true);
    } else {
      e = Eigen::Triplet<bool>(outer_idx_, edges_[outer_idx_][inner_idx_],
                               true);
    }
    return e;
  }

  const Eigen::Triplet<bool>* operator->() const { return &**this; }

  // Advance the iterator to the next edge.
  EdgesIterator& operator++() {
    /* Each edge in `edges` produces a lower- and upper-triplet. In that order.
    So, we flip the upper/lower bit on each advance. When flipping to lower,
    it's time to advance to the next edge in `edges`. */
    upper_triangle_ = !upper_triangle_;
    if (!upper_triangle_) {
      if (++inner_idx_ >= ssize(edges_[outer_idx_])) {
        ++outer_idx_;
        inner_idx_ = 0;
        skip_empty_outer();
      }
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
  int outer_idx_{0};
  int inner_idx_{0};
  bool upper_triangle_{false};
};

}  // namespace

Eigen::SparseMatrix<bool> VisibilityGraph(
    const CollisionChecker& checker,
    const Eigen::Ref<const Eigen::MatrixXd>& points, const bool parallelize) {
  DRAKE_THROW_UNLESS(checker.plant().num_positions() == points.rows());

  const int num_points = points.cols();
  const int num_threads_to_use = (IsOmpEnabledInBuild() && parallelize)
                                     ? checker.num_allocated_contexts()
                                     : 1;
  drake::log()->debug("Generating VisibilityGraph using {} threads",
                      num_threads_to_use);

  // Choose std::vector<uint8_t> as a thread-safe data structure for the
  // parallel evaluations.
  std::vector<uint8_t> points_free(num_points, 0x00);
#if defined(_OPENMP)
#pragma omp parallel for num_threads(num_threads_to_use) schedule(static)
#endif
  for (int i = 0; i < num_points; ++i) {
    points_free[i] = static_cast<uint8_t>(checker.CheckConfigCollisionFree(
        points.col(i), GetContextOmpThreadNum()));
  }

  // Choose std::vector as a thread-safe data structure for the parallel
  // evaluations.
  std::vector<std::vector<int>> edges(num_points);
#if defined(_OPENMP)
#pragma omp parallel for num_threads(num_threads_to_use) schedule(dynamic)
#endif
  for (int i = 0; i < num_points; ++i) {
    if (points_free[i] > 0) {
      edges[i].push_back(i);
      for (int j = i + 1; j < num_points; ++j) {
        if (points_free[j] > 0 &&
            checker.CheckEdgeCollisionFree(points.col(i), points.col(j),
                                           GetContextOmpThreadNum())) {
          edges[i].push_back(j);
        }
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
