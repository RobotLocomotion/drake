#include "drake/planning/graph_algorithms/graph_algorithms_internal.h"

namespace drake {
namespace planning {
namespace graph_algorithms {
namespace internal {

// Given the adjacency matrix of a graph G, return the complement graph H. The
// complement graph H has the same vertex set as G, with vertices (i,j) forming
// an edge in H if and only if they do not form an edge in G.
Eigen::SparseMatrix<bool> ComplementaryAdjacencyMatrix(
    const Eigen::Ref<const Eigen::SparseMatrix<bool>>& adjacency_matrix) {
  // Create an identity matrix of
  Eigen::SparseMatrix<bool> identityMatrix(adjacency_matrix.rows(),
                                           adjacency_matrix.cols());
  for (int i = 0; i < adjacency_matrix.rows(); ++i) {
    identityMatrix.insert(i, i) = true;
  }
  identityMatrix.makeCompressed();

  return !adjacency_matrix - identityMatrix;
};
}  // namespace internal
}  // namespace graph_algorithms
}  // namespace planning
}  // namespace drake
