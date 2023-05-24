// @file
// Utilities for computing the Minimum Degree ordering of a symmetric block
// sparse matrix.

#pragma once

#include <vector>

#include "drake/multibody/contact_solvers/block_sparse_lower_triangular_or_symmetric_matrix.h"

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace internal {

/* Returns the union of a and b (a ∪ b) as a sorted vector.
 @pre a and b are sorted in increasing order. */
std::vector<int> Union(const std::vector<int>& a, const std::vector<int>& b);

/* Returns the set difference of a and b (a \ b) as a sorted vector.
 @pre a and b are sorted in increasing order. */
std::vector<int> SetDifference(const std::vector<int>& a,
                               const std::vector<int>& b);

/* Removes `value` from `sorted_vector` if it exists.
 @pre `sorted_vector` is sorted in increasing order. */
void RemoveValueFromSortedVector(int value, std::vector<int>* sorted_vector);

/* Inserts `value` into `sorted_vector` if it doesn't already exists while
 preserving the order of `sorted_vector`.
 @pre `sorted_vector` is sorted in increasing order. */
void InsertValueInSortedVector(int value, std::vector<int>* sorted_vector);

/* Data structure for a supervariable in an elimination graph as described in
 Algorithm 1 in [Amestoy, 1996]. The algorithm is slightly modified such that
 the supervariables are prescribed and stored in A and L (see below) instead of
 detected using hash functions because the problem we are solving already comes
 in block form.

 [Amestoy 1996] Amestoy, Patrick R., Timothy A. Davis, and
 Iain S. Duff. "An approximate minimum degree ordering algorithm." SIAM Journal
 on Matrix Analysis and Applications 17.4 (1996): 886-905. */
struct Node {
  /* Computes and updates the external degree of `this` node given all nodes in
   the graph. */
  void UpdateExternalDegree(const std::vector<Node>& nodes);

  int degree{0};  // The external degree of the supervaiable.
  int size{0};    // The number of simple variables in a supervariable.
  int index{-1};  // The index of this supervariable/element.
  /* Invariant: A, E, L are sorted. */
  std::vector<int> A;  // Adjacent supervariables when `this` node is a
                       // variable, empty otherwise.
  std::vector<int> E;  // Adjacent elements.
  // TODO(xuchenhan-tri): We can reuse the storage of A for L as only one is
  // non-empty at a given time.
  std::vector<int> L;  // Adjacent supervariables when `this` node is an
                       // element, empty otherwise.
};

/* A simplified version of Node used in a minimum priority queue of that only
 contains index of the node and the degree. */
struct IndexDegree {
  int degree{};
  int index{};
};

/* Comparison function to facilitate a minimum priority queue. */
inline bool operator>(const IndexDegree& a, const IndexDegree& b) {
  if (a.degree != b.degree)
    return a.degree > b.degree;
  else
    return a.index > b.index;
}

/* Computes the preferred elimination ordering of the matrix with the given
 `block_sparsity_pattern` according to the Minimum Degree algorithm. For
 example, a return value of [1, 3, 0, 2] first eliminates block 1, then 3,
 0, and 2. In other words, this is a permutation mapping from new block indices
 to original block indices. */
std::vector<int> CalcEliminationOrdering(
    const BlockSparsityPattern& block_sparsity_pattern);

/* Given the block sparsity pattern of a symmetric block sparse matrix, computes
 the block sparsity pattern of the lower triangular matrix of the resulting from
 a Cholesky factorization.
 @param[in] pattern  The block sparsity pattern of a symmetric block sparse
 matrix. */
BlockSparsityPattern SymbolicFactor(const BlockSparsityPattern& block_sparsity);

}  // namespace internal
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake
