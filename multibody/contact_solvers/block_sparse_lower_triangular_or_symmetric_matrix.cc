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
      blocks_(block_cols_),
      starting_cols_(block_cols_, 0),
      block_row_to_flat_(block_cols_, std::vector<int>(block_cols_, -1)) {
  for (int j = 1; j < block_cols_; ++j) {
    starting_cols_[j] =
        starting_cols_[j - 1] + sparsity_pattern_.block_sizes()[j - 1];
  }
  cols_ = block_cols_ == 0
              ? 0
              : starting_cols_.back() + sparsity_pattern_.block_sizes().back();

  for (int j = 0; j < block_cols_; ++j) {
    blocks_[j].reserve(ssize(block_row_indices(j)));
    for (int flat = 0; flat < ssize(block_row_indices(j)); ++flat) {
      const int i = block_row_indices(j)[flat];
      DRAKE_DEMAND(i >= j);
      block_row_to_flat_[j][i] = flat;

      const int rows = sparsity_pattern_.block_sizes()[i];
      const int cols = sparsity_pattern_.block_sizes()[j];
      blocks_[j].emplace_back(MatrixType::Zero(rows, cols));
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
  MatrixX<double> result = MatrixX<double>::Zero(rows(), cols());
  for (int j = 0; j < block_cols_; ++j) {
    for (int flat = 0; flat < ssize(block_row_indices(j)); ++flat) {
      const int i = block_row_indices(j)[flat];
      const int rows = sparsity_pattern_.block_sizes()[i];
      const int cols = sparsity_pattern_.block_sizes()[j];
      const int starting_row = starting_cols_[i];
      const int starting_col = starting_cols_[j];
      result.block(starting_row, starting_col, rows, cols) = blocks_[j][flat];
      if (i != j && is_symmetric) {
        result.block(starting_col, starting_row, cols, rows) =
            blocks_[j][flat].transpose();
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
