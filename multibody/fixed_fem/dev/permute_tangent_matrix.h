#pragma once

#include <vector>

#include <Eigen/SparseCore>

#include "drake/common/default_scalars.h"
#include "drake/common/drake_assert.h"

namespace drake {
namespace multibody {
namespace fem {
namespace internal {

/* Reindexes the deformable dofs so that the tangent matrix assumes a block
 structure associated with the contact pattern.

 In the context of deformable contact, we may reindex the deformable dofs so
 that the tangent matrix (see FemModel) forms a particular pattern that
 facilitates subsequent computations. The reindexing is done by reindexing the
 vertices with the given `vertex_permutation` and keeping the vertex indexed `i`
 correspond to dofs `3*i`, `3*i+1`, and `3*i+2`.

 For example, suppose the input `tangent matrix is given by
        a a a b b b
        a a a b b b
        a a a b b b
        c c c d d d
        c c c d d d
        c c c d d d
 and the input `vertex permutation` is {1, 0}, then the result of the
 permutation will be:
        d d d c c c
        d d d c c c
        d d d c c c
        b b b a a a
        b b b a a a
        b b b a a a

 @param[in] tangent_matrix      The tangent matrix calculated with the original
                                deformable dof indexes.
 @param[in] vertex_permutation  vertex_permutation[i] gives the permuted index
                                of the vertex whose original index is `i`.
 @pre tangent_matrix.rows() == tangent_matrix.cols().
 @pre tangent_matrix.rows() % 3 == 0.
 @pre vertex_permutation is a permutation of
 {0, 1, ..., tangent_matrix.cols()/3-1}. */
template <typename T>
Eigen::SparseMatrix<T> PermuteTangentMatrix(
    const Eigen::SparseMatrix<T>& tangent_matrix,
    const std::vector<int>& vertex_permutation);

}  // namespace internal
}  // namespace fem
}  // namespace multibody
}  // namespace drake
