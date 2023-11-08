#pragma once

#include <memory>
#include <vector>

#include <Eigen/Sparse>

namespace drake {
namespace planning {
namespace graph_algorithms {
namespace internal {

// Given the adjacency matrix of a graph G, return the complement graph H. The
// complement graph H has the same vertex set as G, with vertices (i,j) forming
// an edge in H if and only if they do not form an edge in G. Note that we
// always exclude self-loops i.e. the diagonal of the returned matrix is always
// false.
Eigen::SparseMatrix<bool> ComplementAdjacencyMatrix(
    const Eigen::Ref<const Eigen ::SparseMatrix<bool>>& adjacency_matrix);

// Given a list of triplets, add the symmetric component to the triplet list.
// This is useful when constructing adjacency matrices.
void SymmetrizeTripletList(std::vector<Eigen::Triplet<bool>>* expected_entries);

}  // namespace internal
}  // namespace graph_algorithms
}  // namespace planning
}  // namespace drake
