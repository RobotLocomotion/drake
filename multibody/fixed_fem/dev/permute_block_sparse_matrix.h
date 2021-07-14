#pragma once

#include <vector>

#include <Eigen/SparseCore>

namespace drake {
namespace multibody {
namespace fem {
namespace internal {

/* Given an Eigen::SparseMatrix with block structure with 3x3 block entries Bᵢⱼ
where i, j ∈ V = {0,...,num_vertices-1} and a permutation P on V, this method
builds the permuted matrix with 3x3 block entries Cᵢⱼ such that Cᵢⱼ =
B_{P(i),P(j)}.

For example, suppose the input `matrix` is given by
       a a a b b b
       a a a b b b
       a a a b b b
       c c c d d d
       c c c d d d
       c c c d d d
and the input `block_permutation` is {1, 0}, then the result of the
permutation will be:
       d d d c c c
       d d d c c c
       d d d c c c
       b b b a a a
       b b b a a a
       b b b a a a

@param[in] matrix             The original block sparse matrix to be permuted.
@param[in] block_permutation  block_permutation[i] gives the permuted index
                              of the block whose original index is `i`.
@pre matrix.rows() == matrix.cols().
@pre matrix.rows() % 3 == 0.
@pre block_permutation is a permutation of {0, 1, ..., matrix.rows()/3-1}. */
template <typename T>
Eigen::SparseMatrix<T> PermuteBlockSparseMatrix(
    const Eigen::SparseMatrix<T>& matrix,
    const std::vector<int>& block_permutation);

}  // namespace internal
}  // namespace fem
}  // namespace multibody
}  // namespace drake
