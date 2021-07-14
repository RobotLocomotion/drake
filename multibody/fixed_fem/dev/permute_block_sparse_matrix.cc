#include "drake/multibody/fixed_fem/dev/permute_block_sparse_matrix.h"

#include "drake/common/default_scalars.h"
#include "drake/common/drake_assert.h"

namespace drake {
namespace multibody {
namespace fem {
namespace internal {

template <typename T>
Eigen::SparseMatrix<T> PermuteBlockSparseMatrix(
    const Eigen::SparseMatrix<T>& matrix,
    const std::vector<int>& block_permutation) {
  DRAKE_ASSERT(matrix.rows() == matrix.cols());
  DRAKE_ASSERT(static_cast<int>(block_permutation.size()) * 3 == matrix.cols());
  const int nv = matrix.rows();
  Eigen::SparseMatrix<T> permuted_matrix(nv, nv);
  std::vector<Eigen::Triplet<T>> triplets;
  triplets.reserve(matrix.nonZeros());
  using InnerIterator = typename Eigen::SparseMatrix<T>::InnerIterator;
  for (int i = 0; i < matrix.outerSize(); ++i) {
    for (InnerIterator it(matrix, i); it; ++it) {
      const int block_row = it.row() % 3;
      const int block_col = it.col() % 3;
      const int block_row_offset = it.row() / 3;
      const int block_col_offset = it.col() / 3;
      const int permuted_row_block = block_permutation[block_row_offset];
      const int permuted_col_block = block_permutation[block_col_offset];
      const int permuted_row = permuted_row_block * 3 + block_row;
      const int permuted_col = permuted_col_block * 3 + block_col;
      triplets.emplace_back(permuted_row, permuted_col, it.value());
    }
  }
  /* The permuted matrix should have the exact same number of non-zero entries
   as the old matrix. */
  DRAKE_DEMAND(static_cast<int>(triplets.size()) == matrix.nonZeros());
  permuted_matrix.setFromTriplets(triplets.begin(), triplets.end());
  return permuted_matrix;
}

DRAKE_DEFINE_FUNCTION_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    (&PermuteBlockSparseMatrix<T>))

}  // namespace internal
}  // namespace fem
}  // namespace multibody
}  // namespace drake
