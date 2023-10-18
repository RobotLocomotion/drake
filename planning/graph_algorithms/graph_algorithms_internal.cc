#include "drake/planning/graph_algorithms/graph_algorithms_internal.h"

#include "drake/common/drake_assert.h"
namespace drake {
namespace planning {
namespace graph_algorithms {
namespace internal {

// Given the adjacency matrix of a graph G, return the complement graph H. The
// complement graph H has the same vertex set as G, with vertices (i,j) forming
// an edge in H if and only if they do not form an edge in G.
Eigen::SparseMatrix<bool> ComplementaryAdjacencyMatrix(
    const Eigen::Ref<const Eigen::SparseMatrix<bool>>& adjacency_matrix) {
  const int n = adjacency_matrix.rows();
  DRAKE_DEMAND(adjacency_matrix.cols() == n);
  std::vector<Eigen::Triplet<bool>> complementary_entries;
  complementary_entries.reserve(n * n - adjacency_matrix.nonZeros() - n);
  for (int i = 0; i < n; ++i) {
    for (int j = i + 1; j < n; ++j) {
      if (!adjacency_matrix.coeff(i, j)) {
        complementary_entries.emplace_back(i, j, true);
        complementary_entries.emplace_back(j, i, true);
      }
    }
  }

  Eigen::SparseMatrix<bool> ret(n, n);
  ret.setFromTriplets(complementary_entries.begin(),
                      complementary_entries.end());
  return ret;
}

// Given a list of triplets, add the symmetric component to the triplet list.
// This is useful when constructing adjacency matrices.
void SymmetrizeTripletList(
    std::vector<Eigen::Triplet<bool>>* expected_entries) {
  expected_entries->reserve(static_cast<int>(2 * expected_entries->size()));
  for (const auto& elt : *expected_entries) {
    expected_entries->emplace_back(elt.col(), elt.row(), elt.value());
  }
}

}  // namespace internal
}  // namespace graph_algorithms
}  // namespace planning
}  // namespace drake
