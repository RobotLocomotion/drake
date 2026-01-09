#pragma once

#include <algorithm>
#include <optional>
#include <set>
#include <utility>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"

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
   @pre i exists in neighbors[i] and the minimum size of `neighbors` is one
        (when only the diagonal block exists). */
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
   `neighbors()[j][k]` gives the block row index of the k-th non-zero block in
   the j-th block column. `neighbors()[j]` is sorted for each block column j and
   each entry in `neighbors()[j]` is greater than or equal to j. In other words,
   only the lower triangular part of the sparsity pattern is specified. As a
   result, we have `neighbors[j][0] = j` for each j because all diagonal blocks
   are nonzero. */
  const std::vector<std::vector<int>>& neighbors() const { return neighbors_; }

  /* Returns the number of "non-zero" (in the sense of eigen::SparseMatrix::nnz,
   i.e., stored densely, not necessarily numerically zero
   https://eigen.tuxfamily.org/dox/classEigen_1_1SparseMatrix.html#a03de8b3da2c142ce8698a76123b3e7d3)
   scalar values in `this` block sparsity pattern. Note that only the non-zero
   entries in the lower triangular part of the matrix are included. */
  int CalcNumNonzeros() const;

 private:
  std::vector<int> block_sizes_;
  std::vector<std::vector<int>> neighbors_;
};

/* This class provides a representation for sparse matrices with a structure
 consisting of dense blocks. It is similar to BlockSparseMatrix in that it
 enables efficient algorithms capable of exploiting highly optimized operations
 with dense blocks. It differs from BlockSparseMatrix in a few aspects:
  1. We only store the block lower triangular portion of the matrix with a
     template argument to imply that the matrix is symmetric (i.e. A = Aᵀ and
     Aij = Ajiᵀ where Aij is the i,j-th block of the matrix A).
  2. It allows modification to the data (but not the sparsity pattern) after
     construction. Therefore, it is suitable for storing matrices with constant
     sparsity pattern and mutable data.
  3. This class only allows square matrices and blocks on the diagonal must be
     square too.
 Most callers should use clearer and less verbose typedefs at the bottom of the
 file (e.g. `BlockSparseSymmetricMatrixXd`) rather than this class template
 directly.

 @tparam MatrixType   The Eigen matrix type of each block in block sparse
                      matrix. The only valid options are `Matrix3d`, `MatrixXd`,
                      or `MatrixX<AutoDiffXd>`.
 @pre MatrixType::RowsAtCompileType == MatrixType::ColsAtCompileTime.
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

  static_assert(MatrixType::RowsAtCompileTime == MatrixType::ColsAtCompileTime);

  using Scalar = typename MatrixType::Scalar;

  /* Constructs a BlockSparseLowerTriangularOrSymmetricMatrix with the given
   block sparsity pattern.
   @param sparsity_pattern  The block sparsity pattern of lower triangular part
                            of the matrix. */
  explicit BlockSparseLowerTriangularOrSymmetricMatrix(
      BlockSparsityPattern sparsity_pattern);

  int rows() const { return cols_; }
  int cols() const { return cols_; }
  int block_rows() const { return block_cols_; }
  int block_cols() const { return block_cols_; }

  /* Adds Aij to the ij-th block of this matrix. If `this` matrix is symmetric,
   Aijᵀ is also implicitly added to the ji-th block of `this` matrix to preserve
   symmetry.
   @pre Aij = Aij.transpose() if i==j and `this` matrix is symmetric.
   @pre i >= j and HasBlock(i, j) == true.
   @note the entire matrix is used if HasBlock(i, j) == true.
   @pre The size of Aij is compatible to the size of ij-th block implied by the
   sparsity pattern at construction. */
  void AddToBlock(int i, int j, const Eigen::Ref<const MatrixType>& Aij) {
    DRAKE_ASSERT_VOID(AssertValid(i, j, Aij, __func__));
    const int flat = block_row_to_flat_[j][i];
    MatrixType& old_value = blocks_[j][flat];
    DRAKE_ASSERT(old_value.rows() == Aij.rows());
    DRAKE_ASSERT(old_value.cols() == Aij.cols());
    old_value += Aij;
  }

  /* Overwrites the ij-th block with `Aij`. If `this` matrix is symmetric,
   Aijᵀ is also implicitly overwritten to the ji-th block of `this` matrix to
   preserve symmetry.
   @pre Aij = Aij.transpose() if i==j and `this` matrix is symmetric.
   @pre i >= j and HasBlock(i, j) == true.
   @pre The size of Aij is compatible to the size of ij-th block implied by the
   sparsity pattern at construction. */
  void SetBlock(int i, int j, MatrixType Aij) {
    DRAKE_ASSERT_VOID(AssertValid(i, j, Aij, __func__));
    const int flat = block_row_to_flat_[j][i];
    SetBlockFlat(flat, j, std::move(Aij));
  }

  /* (Advanced) Similar to SetBlock, but uses flat indices instead of block row
   indices. This is convenient when one has the flat index of a block readily
   available (e.g. while looping over a column when the number of nonzero blocks
   in the column is known).
   @pre 0 <= j < block_cols().
   @pre Aij = Aij.transpose() if the entry corresponds to a diagonal block and
   `this` matrix is symmetric.
   @pre The size of Aij is compatible to the size of ij-th block implied by the
   sparsity pattern at construction.
   @pre The j-th block column has at least `flat+1` nonzero entries. */
  void SetBlockFlat(int flat, int j, MatrixType Aij) {
    DRAKE_ASSERT(0 <= j && j < block_cols_);
    DRAKE_ASSERT(flat >= 0 && flat < ssize(blocks_[j]));
    DRAKE_ASSERT_VOID(
        AssertValid(block_row_indices(j)[flat], j, Aij, __func__));
    MatrixType& old_value = blocks_[j][flat];
    DRAKE_ASSERT(old_value.rows() == Aij.rows());
    DRAKE_ASSERT(old_value.cols() == Aij.cols());
    old_value = std::move(Aij);
  }

  /* Sets the numerical values of all nonzero blocks to zero without changing
   the sparsity pattern. */
  void SetZero();

  /* Makes a dense representation of the matrix. Useful for debugging purposes.
   */
  MatrixX<Scalar> MakeDenseMatrix() const;

  /* Makes a dense representation of the bottom right `num_blocks` blocks of the
   matrix.
   @pre 0 <= num_blocks <= block_cols(). */
  MatrixX<Scalar> MakeDenseBottomRightCorner(int num_blocks) const;

  /* Returns true if the ij-th block in this block sparse matrix is non-zero. In
   particular, this returns false if the indices provided are out of range. */
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
   @pre i >= j and HasBlock(i,j) == true.
   @note To obtain the value of the ij-th block where i < j, if the matrix is
   block lower triangular, then the ij-th block is zero; if the matrix is
   symmetric, one should obtain the Aij via Ajiᵀ. */
  const MatrixType& block(int i, int j) const {
    DRAKE_ASSERT_VOID(AssertValid(i, j, std::nullopt, __func__));
    return blocks_[j][block_row_to_flat_[j][i]];
  }

  /* Returns the j-th diagonal block.
   @pre 0 <= j < block_cols(). */
  const MatrixType& diagonal_block(int j) const {
    DRAKE_ASSERT(0 <= j && j < block_cols_);
    /* Since block_rows are sorted with in each block column, the first entry is
     necessarily the diagonal. */
    return blocks_[j][0];
  }

  /* (Advanced) Similar to `block`, but returns matrix blocks based on flat
   indices instead of block row indices.
   @pre 0 <= j < block_cols().
   @pre The j-th block column has at least `flat+1` nonzero entries. */
  const MatrixType& block_flat(int flat, int j) const {
    DRAKE_ASSERT(0 <= j && j < block_cols_);
    DRAKE_ASSERT(flat >= 0 && flat < ssize(blocks_[j]));
    return blocks_[j][flat];
  }

  /* Returns all stored blocks of `this` matrix. */
  const std::vector<std::vector<MatrixType>>& blocks() const { return blocks_; }

  /* Returns the mapping from block row index to flat index for each column;
   i.e., blocks()[j][block_row_to_flat()[j][i]] gives the (i,j) block.
   block_row_to_flat()[j][i] == -1 if the implied block is empty or is the
   reflection of the symmetric block. */
  const std::vector<std::vector<int>>& block_row_to_flat() const {
    return block_row_to_flat_;
  }

  /* Returns the sorted block row indices in the lower triangular part of the
   j-th block column. This is essentially a map between flat indices to block
   row indices, i.e. `block_row_indices(j)[flat] == i`, where i and j are block
   row and column indices respectively.
   @pre 0 <= j < block_cols(). */
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

  /* If this matrix is symmetric, zeros out all row and column blocks whose
   block indices are included in `indices` and then sets the off-diagonal
   entries of the associated diagonal blocks to zero. This function doesn't
   change the sparsity pattern as it's forbidden by this class (see class
   documentation).
   @pre All entries in `indices` are in [0, block_cols()).
   @throws if `this` matrix is block lower triangular. */
  void ZeroRowsAndColumns(const std::set<int>& indices);

 private:
  /* Checks if the input block row and column indices and optionally the
   corresponding matrix block complies with the requirement of the class; throws
   an exception if not. Invocations provide a string explaining the origin of
   the violation. These checks are expensive and are not suitable for inner loop
   operations common to this class, so we only run these checks in debug builds.
  */
  void AssertValid(int i, int j, const std::optional<MatrixType>& Aij,
                   const char* source) const;

  BlockSparsityPattern sparsity_pattern_;
  /* The number of block columns. */
  int block_cols_{};
  /* The number of columns. */
  int cols_{};
  /* Dense blocks stored in a 2d vector. The first index is the block column
   index and the second index is a flat index that can be retrieved from
   block_row_to_flat_. */
  std::vector<std::vector<MatrixType>> blocks_;
  // TODO(xuchenhan-tri): consider putting the following index book-keeping into
  // the BlockSparsityPattern class.
  /* See getter. */
  std::vector<int> starting_cols_;
  /* Mapping from block row index to flat index for each column; i.e.,
   blocks_[j][block_row_to_flat_[j][i]] gives the (i,j) block.
   block_row_to_flat_[j][i] == -1 if the implied block is empty or is the
   reflection of the symmetric block. */
  // TODO(xuchenhan-tri): consider using
  // std::vector<unordered_map<int, int>> to accomodate large matrices.
  std::vector<std::vector<int>> block_row_to_flat_;
};

template <typename Block>
using BlockSparseLowerTriangularMatrix =
    BlockSparseLowerTriangularOrSymmetricMatrix<Block, false>;

template <typename Block>
using BlockSparseSymmetricMatrix =
    BlockSparseLowerTriangularOrSymmetricMatrix<Block, true>;

using BlockSparseLowerTriangularMatrixXd =
    BlockSparseLowerTriangularMatrix<MatrixX<double>>;
using BlockSparseSymmetricMatrixXd =
    BlockSparseSymmetricMatrix<MatrixX<double>>;
using BlockSparseLowerTriangularMatrix3d =
    BlockSparseLowerTriangularMatrix<Matrix3<double>>;
using BlockSparseSymmetricMatrix3d =
    BlockSparseSymmetricMatrix<Matrix3<double>>;

}  // namespace internal
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake
