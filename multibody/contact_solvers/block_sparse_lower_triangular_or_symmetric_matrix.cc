#include "drake/multibody/contact_solvers/block_sparse_lower_triangular_or_symmetric_matrix.h"

#include <algorithm>

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace internal {

template <typename MatrixType, bool is_symmetric>
BlockSparseLowerTriangularOrSymmetricMatrix<MatrixType, is_symmetric>::
    BlockSparseLowerTriangularOrSymmetricMatrix(
        BlockSparsityPattern sparsity_pattern)
    : sparsity_pattern_(std::move(sparsity_pattern)),
      block_cols_(ssize(sparsity_pattern_.block_sizes())),
      starting_cols_(block_cols_, 0),
      blocks_(block_cols_),
      block_row_to_flat_(block_cols_, std::vector<int>(block_cols_, -1)) {
  for (int i = 1; i < block_cols_; ++i) {
    starting_cols_[i] =
        starting_cols_[i - 1] + sparsity_pattern_.block_sizes()[i - 1];
  }
  cols_ = block_cols_ == 0
              ? 0
              : starting_cols_.back() + sparsity_pattern_.block_sizes().back();

  for (int c = 0; c < block_cols_; ++c) {
    blocks_[c].reserve(ssize(block_row_indices(c)));
    for (int index = 0; index < ssize(block_row_indices(c)); ++index) {
      const int r = sparsity_pattern_.neighbors()[c][index];
      DRAKE_DEMAND(r >= c);
      block_row_to_flat_[c][r] = index;

      const int rows = sparsity_pattern_.block_sizes()[r];
      const int cols = sparsity_pattern_.block_sizes()[c];
      blocks_[c].push_back(MatrixType::Zero(rows, cols));
    }
  }
}

template <typename MatrixType, bool is_symmetric>
void BlockSparseLowerTriangularOrSymmetricMatrix<MatrixType,
                                                 is_symmetric>::SetZero() {
  for (int c = 0; c < block_cols_; ++c) {
    for (MatrixType& block : blocks_[c]) {
      block.setZero();
    }
  }
}

template <typename MatrixType, bool is_symmetric>
MatrixX<double> BlockSparseLowerTriangularOrSymmetricMatrix<
    MatrixType, is_symmetric>::MakeDenseMatrix() const {
  MatrixX<double> result = MatrixType::Zero(rows(), cols());
  for (int j = 0; j < block_cols_; ++j) {
    for (int index = 0; index < ssize(block_row_indices(j)); ++index) {
      const int i = sparsity_pattern_.neighbors()[j][index];
      const int rows = sparsity_pattern_.block_sizes()[i];
      const int cols = sparsity_pattern_.block_sizes()[j];
      const int starting_row = starting_cols_[i];
      const int starting_col = starting_cols_[j];
      result.block(starting_row, starting_col, rows, cols) = blocks_[j][index];
      if (i != j && is_symmetric) {
        result.block(starting_col, starting_row, cols, rows) =
            blocks_[j][index].transpose();
      }
    }
  }
  return result;
}

template class BlockSparseLowerTriangularOrSymmetricMatrix<MatrixX<double>,
                                                           true>;
template class BlockSparseLowerTriangularOrSymmetricMatrix<MatrixX<double>,
                                                           false>;
template class BlockSparseLowerTriangularOrSymmetricMatrix<Matrix3<double>,
                                                           true>;
template class BlockSparseLowerTriangularOrSymmetricMatrix<Matrix3<double>,
                                                           false>;

}  // namespace internal
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake
