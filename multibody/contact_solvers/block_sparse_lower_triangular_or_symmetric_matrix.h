#pragma once

#include <algorithm>
#include <utility>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/common/ssize.h"

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace internal {

/* The sparsity pattern of a block sparse matrix encoded as a undirected graph.
 All diagonal blocks of the sparse matrix are guaranteed to be square and
 non-zero. They can be viewed as nodes in the graph. The number of rows (and
 columns) of these diagonal blocks can be queried with `block_sizes()`.

 An edge exist between block i and block j if the ij-th block is non-zero in the
 block sparse matrix. The connectivity information can be queried with
 `neighbors()`. */
class BlockSparsityPattern {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(BlockSparsityPattern);

  /* Constructs a BlockSparsityPattern with the given information about the
   diagonal blocks and the connectivity pattern.
   @param[in] block_sizes  The number of rows (and columns) in the square
                           diagonal blocks.
   @param[in] neighbors    neighbors[i] lists the nodes connected to the node i.
   @pre Each node j in neighbor[i] satisfies j >= i, i.e, if an edge exists
        between i and j, j exists in neighbor[i], but i doesn't exist in
        neighbor[j].
   @pre i exists in neighbors[i]*/
  BlockSparsityPattern(std::vector<int> block_sizes,
                       std::vector<std::vector<int>> neighbors)
      : block_sizes_(std::move(block_sizes)), neighbors_(std::move(neighbors)) {
    DRAKE_DEMAND(block_sizes_.size() == neighbors_.size());
    for (int i = 0; i < ssize(block_sizes_); ++i) {
      /* Sort and remove all duplicates. */
      std::sort(neighbors_[i].begin(), neighbors_[i].end());
      neighbors_[i].erase(
          std::unique(neighbors_[i].begin(), neighbors_[i].end()),
          neighbors_[i].end());
      /* Check invariants. */
      DRAKE_DEMAND(neighbors_[i].size() > 0);
      DRAKE_DEMAND(neighbors_[i][0] == i);
      DRAKE_DEMAND(neighbors_[i].back() < ssize(block_sizes_));
    }
  }

  /* The number of rows (and columns) in each diagonal block of the matrix. */
  const std::vector<int>& block_sizes() const { return block_sizes_; }

  /* The connectivity pattern of the sparsity. More specifically,
   `neighbors()[c][i]` gives the block row index of the i-th non-zero block in
   the c-th block column. `neighbors[c]` is sorted for each block column c and
   that each entry in `neighbors[c]` is greater than or equal to c. In other
   words, only the lower triangular part of the sparsity pattern is specified.
   As a result, we have `neighbors[c][0] = c` for each c because all diagonal
   blocks are nonzero. */
  const std::vector<std::vector<int>>& neighbors() const { return neighbors_; }

 private:
  std::vector<int> block_sizes_;
  std::vector<std::vector<int>> neighbors_;
};

/* This class provides a representation for sparse matrices with a structure
 consisting of dense blocks. It is similar to BlockSparseMatrix in that it
 enables efficient algorithms capable of exploiting highly optimized operations
 with dense blocks. It differs from BlockSparseMatrix in a few aspects:
  1. We only store the lower triangular portion of the matrix with a template
     argument to imply that the matrix is symmetric.
  2. It allows modification to the data (but not the sparsity pattern) after
     construction. Therefore, it is suitable for storing matrices with constant
     sparsity pattern and mutable data.

 @tparam MatrixType   The matrix type of each block in block sparse matrix,
                      Matrix3<double> or Matrix<X<double>.
 @tparam is_symmetric Determines whether the matrix is symmetric or lower
                      triangular.
 @warn Data is ALWAYS retrieved/set through the lower triangular part of the
 matrix, even if the matrix is symmetric. For example, to set the 1,2 component
 of a symmetric matrix to A, one would need to set the 2,1 component to Aᵀ to
 achieve the desired effect. */
template <typename MatrixType, bool is_symmetric>
class BlockSparseLowerTriangularOrSymmetricMatrix {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(
      BlockSparseLowerTriangularOrSymmetricMatrix);

  /* Constructs a BlockSparseLowerTriangularOrSymmetricMatrix with the given
   block sparsity pattern.
   @param sparsity_pattern  The block sparsity pattern of lower triangular part
                            of the matrix. */
  BlockSparseLowerTriangularOrSymmetricMatrix(
      BlockSparsityPattern sparsity_pattern);

  int rows() const { return cols_; }
  int cols() const { return cols_; }
  int block_rows() const { return block_cols_; }
  int block_cols() const { return block_cols_; }

  /* Adds Aij to the ij-th block of this matrix.
   @pre Aij = Aij.transpose() if i==j and `this` matrix is symmetric.
   @pre HasBlock(i, j) == true.
   @pre The size of Aij is compatible to the size of ij-th block implied by the
   sparsity pattern at construction. */
  void AddToBlock(int i, int j, const Eigen::Ref<const MatrixType>& Aij) {
    DRAKE_ASSERT(HasBlock(i, j));
    const int flat = block_row_to_flat_[j][i];
    MatrixType& old_value = blocks_[j][flat];
    DRAKE_ASSERT(old_value.rows() == Aij.rows());
    DRAKE_ASSERT(old_value.cols() == Aij.cols());
    old_value += Aij;
  }

  /* Similar to AddToBlock, but overwrites instead of accumulates.
   @pre Aij = Aij.transpose() if i==j and `this` matrix is symmetric.
   @pre HasBlock(i, j) == true.
   @pre The size of Aij is compatible to the size of ij-th block implied by the
   sparsity pattern at construction. */
  void SetBlock(int i, int j, MatrixType Aij) {
    DRAKE_ASSERT(HasBlock(i, j));
    const int flat = block_row_to_flat_[j][i];
    MatrixType& old_value = blocks_[j][flat];
    DRAKE_ASSERT(old_value.rows() == Aij.rows());
    DRAKE_ASSERT(old_value.cols() == Aij.cols());
    old_value = std::move(Aij);
  }

  /* (Advanced) Similar to SetBlock, but uses flat indices instead of block row
   indices. This is convenient when one has the flat index of block readily
   available. */
  void SetBlockFlat(int flat, int j, MatrixType Aij) {
    DRAKE_ASSERT(flat >= 0 && flat < ssize(blocks_[j]));
    blocks_[j][flat] = std::move(Aij);
  }

  /* Sets the numerical values of all nonzero blocks to zero without
   changing the sparsity pattern. */
  void SetZero();

  /* Makes a dense representation of the matrix. Useful for debugging purpose.
   */
  MatrixX<double> MakeDenseMatrix() const;

  /* Returns true if the ij-th block in this block sparse matrix is non-zero. */
  bool HasBlock(int i, int j) const {
    if (j > i) {
      return is_symmetric ? HasBlock(j, i) : false;
    }
    if (i < 0 || i >= block_rows() || j < 0 || j >= block_cols()) {
      return false;
    }
    return block_row_to_flat_[j][i] >= 0;
  }

  /* Returns the ij-th block.
   @pre HasBlock(i,j) == true. */
  const MatrixType& block(int i, int j) const {
    DRAKE_ASSERT(HasBlock(i, j));
    return blocks_[j][block_row_to_flat_[j][i]];
  }

  /* Returns the mutable ij-th block.
   @pre HasBlock(i,j) == true. */
  MatrixType& mutable_block(int i, int j) {
    DRAKE_ASSERT(HasBlock(i, j));
    return blocks_[j][block_row_to_flat_[j][i]];
  }

  /* Returns the i-th diagonal block. */
  const MatrixType& diagonal_block(int i) const {
    DRAKE_ASSERT(0 <= i && i < block_cols_);
    /* Since block_rows are sorted with in each block column, the first entry is
     necessarily the diagonal. */
    return blocks_[i][0];
  }

  /* (Advanced) Similar to `block`, but returns matrix blocks based on flat
   indices instead of block row indices. */
  const MatrixType& block_flat(int flat, int j) {
    DRAKE_ASSERT(0 <= j && j < block_cols_);
    return blocks_[j][flat];
  }

  /* (Advanced) Similar to `mutable_block`, but returns matrix blocks based on
   flat indices instead of block row indices. */
  MatrixType& mutable_block_flat(int flat, int j) {
    DRAKE_ASSERT(0 <= j && j < block_cols_);
    return blocks_[j][flat];
  }

  /* Returns the sorted block row indices in the lower triangular part of the
   j-th block column. */
  const std::vector<int>& block_row_indices(int j) const {
    DRAKE_ASSERT(0 <= j && j < block_cols_);
    return sparsity_pattern_.neighbors()[j];
  }

  /* Returns the sparsity pattern of the matrix. */
  const BlockSparsityPattern& sparsity_pattern() const {
    return sparsity_pattern_;
  }

  /* Returns the indices of the starting scalar column of each block column. If
   the block columns are of size 2, 3, 11, 5, then this function returns
   [0, 2, 5, 16]. */
  const std::vector<int>& starting_cols() const { return starting_cols_; }

 private:
  BlockSparsityPattern sparsity_pattern_;
  /* The number of block columns. */
  int block_cols_{};
  /* The number of columns. */
  int cols_{};
  /* See getter. */
  std::vector<int> starting_cols_;
  /* Dense blocks stored in a 2d vector. The first index is the block column
   index and the second index is a flat index that can be retrieved from
   block_row_to_flat_. */
  std::vector<std::vector<MatrixType>> blocks_;
  /* Mapping from block row index to flat index for each column; i.e.,
   blocks_[c][block_row_to_flat_[c][r]] gives the (r,c) block.
   block_row_to_flat_[c][r] == -1 if the implied block is empty. */
  std::vector<std::vector<int>> block_row_to_flat_;
};

using BlockSparseLowerTriangularMatrix =
    BlockSparseLowerTriangularOrSymmetricMatrix<MatrixX<double>, false>;
using BlockSparseSymmetricMatrix =
    BlockSparseLowerTriangularOrSymmetricMatrix<MatrixX<double>, true>;
using Block3x3SparseLowerTriangularMatrix =
    BlockSparseLowerTriangularOrSymmetricMatrix<Matrix3<double>, false>;
using Block3x3SparseSymmetricMatrix =
    BlockSparseLowerTriangularOrSymmetricMatrix<Matrix3<double>, true>;

}  // namespace internal
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake
