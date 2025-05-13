// @file
// Utilities for computing the Minimum Degree ordering of a symmetric block
// sparse matrix.

#pragma once

#include <unordered_set>
#include <vector>

#include "drake/multibody/contact_solvers/block_sparse_lower_triangular_or_symmetric_matrix.h"

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace internal {

/* Performs an in-place sorted union by merging b into a.
 @pre a and b are sorted in increasing order. */
void InplaceSortedUnion(const std::vector<int>& b, std::vector<int>* a);

/* Performs an in-place set difference of a and b, (a \ b), into a.
 @pre a and b are sorted in increasing order. */
void InplaceSortedDifference(const std::vector<int>& b, std::vector<int>* a);

/* Removes `value` from `sorted_vector` if it exists.
 @pre `sorted_vector` is sorted in increasing order. */
void RemoveValueFromSortedVector(int value, std::vector<int>* sorted_vector);

/* Inserts `value` into `sorted_vector` if it doesn't already exists while
 preserving the order of `sorted_vector`.
 @pre `sorted_vector` is sorted in increasing order. */
void InsertValueInSortedVector(int value, std::vector<int>* sorted_vector);

/* Data structure for a node in an elimination graph as described in Algorithm 1
 in [Amestoy, 1996] and section 7.1 in [Davis 2006]. The algorithm is slightly
 modified such that the supervariables are prescribed and stored in A and L (see
 below) instead of detected using hash functions because the problem we are
 solving already comes in block form. In addition, each node is already a
 supervariable at the start of the elimination. Essentially we do not implement
 the merging of nodes in the graph as explained in [Amestoy, 1996. §3.2].

 [Amestoy 1996] Amestoy, Patrick R., Timothy A. Davis, and
 Iain S. Duff. "An approximate minimum degree ordering algorithm." SIAM Journal
 on Matrix Analysis and Applications 17.4 (1996): 886-905.
 [Davis 2006] Davis, Timothy A. Direct methods for sparse linear systems.
 Society for Industrial and Applied Mathematics, 2006. */
struct Node {
  /* Computes and updates the external degree of `this` node given all nodes in
   the graph.
   @param[in] nodes  All nodes in the graph.
   @param[in] seen   pre-allocated scratch buffer, used to mark nodes
                     encountered during update.
   @pre `seen` has all zero values and is of the same size as `nodes`.
   @post `seen` is reset to all zeros. */
  void UpdateExternalDegree(const std::vector<Node>& nodes,
                            std::vector<uint8_t>* seen);

  /* Given all nodes in the graph, approximates and updates the external degree
   of `this` node when its neighboring node p is eliminated.
   @param[in] p        The index of node being eliminated (which is neighboring
                       this node).
   @param[in] Lp_size  |Lp \ i| the second term in equation 4. [Amestoy 1996].
   @param[in] nodes    All nodes in the graph. */
  void ApproximateExternalDegree(int p, int Lp_size,
                                 const std::vector<Node>& nodes);

  /* See [Amestoy 1996] and [Davis 2006] for definitions of "supervariable",
   "element", "external degree", etc. */
  int degree{0};  // The external degree of the supervaiable. Note that this is
                  // not simply the number of neighboring nodes, but the
                  // summation of block sizes on the neighbors because the
                  // neighboring nodes are supervariables themselves.
  int size{0};    // The number of simple variables in a supervariable.
  int index{-1};  // The index of this node that can be used as an unique
                  // identifier of the node.
  /* Invariant: A, E, L are sorted. We opt for a std::vector over std::set for
   slightly lower overhead when performing union operations for these sets. */
  std::vector<int> A;  // Adjacent supervariables when `this` node is a
                       // variable; empty when this node is an element.
  std::vector<int> E;  // Adjacent elements when `this` node is a variable;
                       // empty when this node is an element due to absorption.
  // TODO(xuchenhan-tri): We can reuse the storage of A for L as only one is
  // non-empty at a given time.
  std::vector<int> L;  // Adjacent supervariables when `this` node is an
                       // element, empty otherwise.
  int weight;  // Weight of this node (`w` in Algorithm 2 in [Amestoy 1996])
               // when this node is an element, -1 otherwise.
};

/* Updates the weights of all nodes according to Algorithm 2 in [Amestoy 1996].
 @param[in] Lp     The adjacent supervariables of the node being eliminated.
 @param[in] nodes  All nodes in the graph. */
void UpdateWeights(const std::vector<int>& Lp, std::vector<Node>* nodes);

/* Computes the preferred elimination ordering of the matrix with the given
 `block_sparsity_pattern` according to the Minimum Degree algorithm. For
 example, a return value of [1, 3, 0, 2] first eliminates block 1, then 3,
 0, and 2. In other words, this is a permutation mapping from new block indices
 to original block indices. */
std::vector<int> ComputeMinimumDegreeOrdering(
    const BlockSparsityPattern& block_sparsity_pattern, bool use_amd = true);

/* Similar to the one argument overload but eliminates elements in
`priority_elements` first.
 Suppose the sparsity pattern has n nodes (which we denote as V) and m of those
 are in `priority_elements` (which we denote as P). Let D(v) be the degree of
 the node v. For k ∈ [0, m), the k-th eliminated node is argmin(D(v)) s.t.
 v ∈ P. For k ∈ [m, n), the k-th eliminated node is argmin(D(v)) s.t. v ∈ V\P.
 @pre all entries in `priority_elements` are in {0, 1, ..., n-1}. */
std::vector<int> ComputeMinimumDegreeOrdering(
    const BlockSparsityPattern& block_sparsity_pattern,
    const std::unordered_set<int>& priority_elements, bool use_amd = true);

/* Given the block sparsity pattern of a symmetric block sparse matrix, computes
 the block sparsity pattern of the lower triangular matrix resulting from a
 Cholesky factorization. Specifically, L(i, j) ≠ 0 if A(i, j) ≠ 0 or if there
 exists k such that A(k, i) and A(k, j) is non-zero ( for k < j ≤ i) We
 implement this efficiently with an elimination tree.
 See section 4.1 in [Davis 2006] for discussion on practical implementation.

 [Davis 2006] Davis, Timothy A. Direct methods for sparse linear systems.
 Society for Industrial and Applied Mathematics, 2006.

 @param[in] pattern The block sparsity pattern of a symmetric block sparse
 matrix. */
BlockSparsityPattern SymbolicCholeskyFactor(
    const BlockSparsityPattern& block_sparsity);

/* Given a block sparsity pattern G on vertices v = {0, 1, ..., n-1} and a
 partition on v = v₁ ∪ v₂ (such that v₁ ∩ v₂ = ∅), computes an elimination
 ordering on v in the following way:
  1. Generate the v₁-induced subgraph G₁ and the v₂-induced subgraph G₂.
  2. Compute the Minimum Degree ordering on G₁ and G₂ respectively.
  3. Concatenate the orderings so that all vertices in v₁ come before vertices
     in v₂.
 @param[in] global_pattern  The block sparsity pattern G.
 @param[in] v1              The vertices in the set v₁.
 @returns  The elimination ordering obtained by following the algorithm
 described above. */
std::vector<int> CalcAndConcatenateMdOrderingWithinGroup(
    const BlockSparsityPattern& global_pattern,
    const std::unordered_set<int>& v1);

}  // namespace internal
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake
