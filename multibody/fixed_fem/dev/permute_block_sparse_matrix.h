#pragma once

#include <vector>

#include <Eigen/SparseCore>

namespace drake {
namespace multibody {
namespace fem {
namespace internal {
// TODO(xuchenhan-tri): Consider moving this method to matrix_utilities.h/cc.
/* Given a 3N-by-3N Eigen::SparseMatrix with block structure with 3x3 block
entries Bᵢⱼ where i, j ∈ V = {0, ..., N-1} and a permutation P on V, this method
builds the permuted matrix with 3x3 block entries C's such that Cₚ₍ᵢ₎ₚ₍ⱼ₎ =
Bᵢⱼ.

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
@param[in] block_permutation  block_permutation[i] gives the index of the
                              permuted block whose original index is `i`.
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
