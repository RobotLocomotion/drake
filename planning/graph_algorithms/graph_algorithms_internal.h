#pragma once

#include <memory>

#include <Eigen/Sparse>

namespace drake {
namespace planning {
namespace internal {

// Given the adjacency matrix of a graph G, return the complement graph H. The
// complement graph H has the same vertex set as G, with vertices (i,j) forming
// an edge in H if and only if they do not form an edge in G.
Eigen::SparseMatrix<bool> ComplementaryAdjacencyMatrix(
    const Eigen::Ref < const Eigen ::SparseMatrix<bool>>& adjacency_matrix);
}  // namespace internal
}  // namespace planning
}  // namespace drake