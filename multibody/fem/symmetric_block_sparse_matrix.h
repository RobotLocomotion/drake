#pragma once

#include <set>
#include <utility>
#include <vector>

#include <Eigen/Sparse>

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"

namespace drake {
namespace multibody {
namespace fem {
namespace internal {

/* The sparsity pattern of a symmetric block sparse matrix. Each diagonal block
 is always non-zero and square and the number of its rows (and cols) is stored
 in `diagonals`. Off diagonal blocks may or may not be nonzero and is dictated
 by `sparsity_pattern`. The number of blocks in column c is given by
 `sparsity_pattern[c].size()`. `sparsity_pattern[c][i]` gives the block row
 index of the i-th block in the c-th block column. In addition,
 sparsity_pattern[c][i] >= c. In other words, only the lower triangular part of
 the sparsity pattern is specified. We require `sparsity_pattern[c]` to be
 sorted for each c. As a result, we have sparsity_pattern[c][0] = c for each c
 since the diagonal block is always nonzero. */
struct BlockSparsityPattern {
  std::vector<int> diagonals;
  std::vector<std::vector<int>> sparsity_pattern;
};

/* This class provides a representation for sparse matrices with a structure
 consisting of dense blocks. It is similar to
 contact_solvers::internal::BlockSparseMatrix in that it enables efficient
 algorithms capable of exploiting highly optimized operations with dense blocks.
 It differs from BlockSparseMatrix in a few aspects:

  2. It is tailored to symmetric matrices and only stores the lower triangular
     part of the matrix.
  3. It allows modification to the data (but not the sparsity pattern) after
     construction. Therefore, it is suitable for storing matrices with constant
     sparsity pattern and mutable data.

 In particular, these features make SymmetricBlockSparseMatrix suitable for
 storing the stiffness/damping/tangent matrix of an FEM model, where the matrix
 has constant sparsity pattern and is symmetric.
 @tparam_nonsymbolic_scalar */
template <typename T>
class SymmetricBlockSparseMatrix {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(SymmetricBlockSparseMatrix);

  /* Constructs a SymmetricBlockSparseMatrix with the given block sparsity
   pattern. */
  explicit SymmetricBlockSparseMatrix(
      BlockSparsityPattern block_sparsity_pattern);

  int rows() const { return cols_; }
  int cols() const { return cols_; }
  int block_rows() const { return block_cols_; }
  int block_cols() const { return block_cols_; }

  /* Adds Aij to the ij-th block of this matrix.
   @pre Aij = Aij.transpose() if i==j.
   @pre The size of Aij is compatible to the ij-th block specified at
   construction. */
  void AddToBlock(int i, int j, const Eigen::Ref<const MatrixX<T>>& Aij);

  /* For the ij-th block M, do M -= A * Bᵀ.
   @pre The size of A * Bᵀ is compatible to the ij-th block specified at
   construction. */
  void SubtractProductFromBlock(int i, int j, const MatrixX<T>& A,
                                const MatrixX<T>& B) {
    DRAKE_ASSERT(A.cols() == B.cols());
    DRAKE_ASSERT(A.rows() == block_sparsity_pattern_.diagonals[i]);
    DRAKE_ASSERT(B.rows() == block_sparsity_pattern_.diagonals[j]);
    const int index = block_row_to_flat_[j][i];
    blocks_[j][index] -= A * B.transpose();
  }

  /* Returns the flat-th block in j-th block_column. */
  const MatrixX<T>& get_block_flat(int flat, int j) const {
    return blocks_[j][flat];
  }

  /* Similar to AddToBlock, but overwrites instead of accumulates. */
  void SetBlock(int i, int j, MatrixX<T> Aij);
  void SetBlockFlat(int flat, int j, MatrixX<T> Aij) {
    blocks_[j][flat] = std::move(Aij);
  }

  void SetZero();

  MatrixX<T> MakeDenseMatrix() const;

  /* Returns true if there exists a ij-th block in this block sparse matrix. */
  bool has_block(int i, int j) const {
    if (i < 0 || i >= block_rows() || j < 0 || j >= block_cols()) {
      return false;
    }
    return block_row_to_flat_[j][i] >= 0;
  }

  /* Returns the ij-th block.
   @pre has_block(i,j) == true. */
  const MatrixX<T>& get_block(int i, int j) const {
    DRAKE_ASSERT(has_block(i, j));
    return blocks_[j][block_row_to_flat_[j][i]];
  }

  /* Returns the i-th diagonal block. */
  const MatrixX<T>& get_diagonal_block(int i) const {
    /* Since block_rows are sorted with in each block column, the first entry is
     necessarily the diagonal. */
    return blocks_[i][0];
  }

  /* Returns the mutable ij-th block.
   @pre has_block(i,j) == true. */
  MatrixX<T>& get_mutable_block(int i, int j) {
    DRAKE_ASSERT(has_block(i, j));
    return blocks_[j][block_row_to_flat_[j][i]];
  }

  const std::vector<int>& get_row_indices_in_col(int j) const {
    DRAKE_DEMAND(0 <= j && j < block_cols_);
    return block_sparsity_pattern_.sparsity_pattern[j];
  }

  // TODO(xuchenhan-tri): Change this to weighted adjacency graph.
  /* Calculates the adjacency graph corresponding to the sparsity pattern of
   this matrix. Note that each vertex in the graph corresponds to a 3x3 diagonal
   block instead of a single entry.
  std::vector<std::set<int>> CalcAdjacencyGraph() const; */

 private:
  friend class BlockSparseCholeskySolver;

  int num_blocks_in_col(int j) const {
    return get_row_indices_in_col(j).size();
  }

  BlockSparsityPattern block_sparsity_pattern_;
  int block_cols_{};
  int cols_{};
  /* The starting column of each column block. */
  std::vector<int> starting_col_;
  /* Dense blocks stored in a 2d vector. The first index is the block column
   index and the second index is a flat index that can be retrieved from
   block_row_to_flat_ below. */
  std::vector<std::vector<MatrixX<T>>> blocks_;
  /* Mapping from block row index to flat index for each column; i.e.,
   blocks_[c][block_row_to_flat_[c][r]] gives the (r,c) block.
   block_row_to_flat_[c][r] == -1 if the implied block is empty. */
  std::vector<std::vector<int>> block_row_to_flat_;
};

}  // namespace internal
}  // namespace fem
}  // namespace multibody
}  // namespace drake
