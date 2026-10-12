#pragma once

#include "drake/multibody/contact_solvers/block_sparse_lower_triangular_or_symmetric_matrix.h"

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace icf {
namespace internal {

/* Hessians used by ICF are always block-sparse and symmetric. */
template <typename T>
using IcfHessian =
    typename contact_solvers::internal::BlockSparseSymmetricMatrix<MatrixX<T>>;

}  // namespace internal
}  // namespace icf
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake
