#include "drake/multibody/fem/symmetric_block_sparse_matrix.h"

#include <algorithm>

#include <fmt/format.h>

namespace drake {
namespace multibody {
namespace fem {
namespace internal {

template <typename T>
SymmetricBlockSparseMatrix<T>::SymmetricBlockSparseMatrix(
    BlockSparsityPattern block_sparsity_pattern)
    : block_sparsity_pattern_(std::move(block_sparsity_pattern)),
      block_cols_(block_sparsity_pattern_.diagonals.size()),
      starting_col_(block_cols_, 0),
      blocks_(block_cols_),
      block_row_to_flat_(block_cols_, std::vector<int>(block_cols_, -1)) {
  for (int i = 1; i < block_cols_; ++i) {
    starting_col_[i] =
        starting_col_[i - 1] + block_sparsity_pattern_.diagonals[i - 1];
  }
  cols_ = starting_col_.back() + block_sparsity_pattern_.diagonals.back();

  for (int c = 0; c < block_cols_; ++c) {
    blocks_[c].reserve(num_blocks_in_col(c));
    for (int index = 0; index < num_blocks_in_col(c); ++index) {
      const int r = block_sparsity_pattern_.sparsity_pattern[c][index];
      DRAKE_DEMAND(r >= c);
      block_row_to_flat_[c][r] = index;

      const int rows = block_sparsity_pattern_.diagonals[r];
      const int cols = block_sparsity_pattern_.diagonals[c];
      blocks_[c].push_back(MatrixX<T>::Zero(rows, cols));
    }
  }
}

template <typename T>
void SymmetricBlockSparseMatrix<T>::AddToBlock(
    int i, int j, const Eigen::Ref<const MatrixX<T>>& Aij) {
  DRAKE_DEMAND(0 <= j && j <= i && i < block_cols_);
  const int index = block_row_to_flat_[j][i];
  DRAKE_DEMAND(index >= 0);
  MatrixX<T>& old_value = blocks_[j][index];
  DRAKE_DEMAND(old_value.rows() == Aij.rows());
  DRAKE_DEMAND(old_value.cols() == Aij.cols());
  old_value += Aij;
}

template <typename T>
void SymmetricBlockSparseMatrix<T>::SetBlock(int i, int j, MatrixX<T> Aij) {
  DRAKE_DEMAND(0 <= j && j <= i && i < block_cols_);
  const int index = block_row_to_flat_[j][i];
  DRAKE_DEMAND(index >= 0);
  MatrixX<T>& old_value = blocks_[j][index];
  DRAKE_DEMAND(old_value.rows() == Aij.rows());
  DRAKE_DEMAND(old_value.cols() == Aij.cols());
  old_value = std::move(Aij);
}

template <typename T>
void SymmetricBlockSparseMatrix<T>::SetZero() {
  for (int c = 0; c < block_cols_; ++c) {
    for (auto& block : blocks_[c]) {
      block.setZero();
    }
  }
}

template <typename T>
MatrixX<T> SymmetricBlockSparseMatrix<T>::MakeDenseMatrix() const {
  MatrixX<T> A = MatrixX<T>::Zero(rows(), cols());
  for (int j = 0; j < block_cols_; ++j) {
    for (int index = 0; index < num_blocks_in_col(j); ++index) {
      const int i = block_sparsity_pattern_.sparsity_pattern[j][index];
      const int rows = block_sparsity_pattern_.diagonals[i];
      const int cols = block_sparsity_pattern_.diagonals[j];
      const int starting_row = starting_col_[i];
      const int starting_col = starting_col_[j];
      A.template block(starting_row, starting_col, rows, cols) =
          blocks_[j][index];
      if (i != j) {
        A.template block(starting_col, starting_row, cols, rows) =
            blocks_[j][index].transpose();
      }
    }
  }
    return A;
}

// template <typename T>
// std::vector<std::set<int>>
// SymmetricBlockSparseMatrix<T>::CalcAdjacencyGraph()
//     const {
//   std::vector<std::set<int>> result;
//   result.reserve(block_cols_);
//   for (const std::vector<int>& neighbors : sparsity_pattern_) {
//     result.emplace_back(std::set<int>(neighbors.begin(), neighbors.end()));
//   }
//   return result;
// }

template class SymmetricBlockSparseMatrix<double>;

}  // namespace internal
}  // namespace fem
}  // namespace multibody
}  // namespace drake
