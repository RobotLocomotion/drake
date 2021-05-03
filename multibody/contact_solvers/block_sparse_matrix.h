#pragma once

#include <tuple>
#include <unordered_set>
#include <utility>
#include <vector>

#include "drake/common/default_scalars.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace internal {

template <typename T>
class BlockSparseMatrixBuilder;

// This class provides a representation for sparse matrices with a structure
// consisting of dense blocks of non-zeros. While other storage formats such as
// CRS (Compressed Row Storage) are popular, a data structure tailored to
// block-sparse matrices enables efficient algorithms capable of exploiting
// highly optimized operations with dense blocks (e.g. via AVX instructions).
//
// Instances of this class are meant to be built with BlockSparseMatrixBuilder
// to ensure the consistency of block entries provided by users.
//
// @tparam_nonsymbolic_scalar
template <typename T>
class BlockSparseMatrix {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(BlockSparseMatrix)

  // A non-zero block entry is specified with the triplet {i, j, Bij}, where
  // (i,j) are the i-th and j-th block row and column respectively and Bij is
  // the dense block entry.
  typedef std::tuple<int, int, MatrixX<T>> BlockTriplet;

  // Constructs a zero sized matrix.
  BlockSparseMatrix() = default;

  int rows() const { return rows_; }
  int cols() const { return cols_; }

  // The number of blocks along the rows dimension. Not necessarily equal to
  // rows().
  int block_rows() const { return block_row_size_.size(); }

  // The number of blocks along the column dimension. Not necessarily equal to
  // cols().
  int block_cols() const { return block_col_size_.size(); }

  // The total number of dense blocks sotred by this class.
  int num_blocks() const { return blocks_.size(); }

  // Access to the b-th block. b must be in the range 0 to num_blocks()-1.
  const MatrixX<T>& get_block(int b) const {
    DRAKE_DEMAND(b < num_blocks());
    return std::get<2>(blocks_[b]);
  }

  // Access to an vector of all triplets stored by this class.
  const std::vector<BlockTriplet>& get_blocks() const { return blocks_; }

  // Size of the i-th block row.
  int block_row_size(int i) const {
    DRAKE_DEMAND(i < block_rows());
    return block_row_size_[i];
  }

  // Size of the i-th block column.
  int block_col_size(int i) const {
    DRAKE_DEMAND(i < block_cols());
    return block_col_size_[i];
  }

  // Returns the index of the row at which the i-th block row starts.
  int row_start(int i) const {
    DRAKE_DEMAND(i < block_rows());
    return row_start_[i];
  }

  // Returns the index of the column at which the i-th block column starts.
  int col_start(int i) const {
    DRAKE_DEMAND(i < block_cols());
    return col_start_[i];
  }

  // For this matrix A, it performs the operation y=A⋅x. x must be of size
  // cols() and y must be a non-nullptr to a vector of size rows().
  void Multiply(const Eigen::Ref<const VectorX<T>>& x,
                EigenPtr<VectorX<T>> y) const;

  // For this matrix A, it performs the operation y=Aᵀ⋅x. x must be of size
  // rows() and y must be a non-nullptr to a vector of size cols().
  void MultiplyByTranspose(const Eigen::Ref<const VectorX<T>>& x,
                           EigenPtr<VectorX<T>> y) const;

  // Makes a dense matrix representation of this block-sparse matrix.
  MatrixX<T> MakeDenseMatrix() const;

 private:
  // Builder needs access to private constructors.
  friend class BlockSparseMatrixBuilder<T>;

  // Constructs a BlockSparseMatrix from a known block-sparse structure.
  // block_row_size[i] stores the size of the i-th block row.
  // block_col_size[j] stores the size of the j-th block column.
  // blocks[b] stores the b-th block triplet (i, j, Aij) where i <
  // block_row_size.size() is the i-th block row, j < block_col_size.size() is
  // the j-th block column and Aij is the block entry of size block_row_size[i]
  // by block_col_size[j].
  // @warning this constructor does not verify the validity of the arguments.
  // That is, it assumes that:
  // - There are no zero sized block rows or columns. That is, all entries in
  // block_row_size and block_col_size are strictly positive.
  // - The block sizes in `blocks` are consistent with block_row_size and
  // block_col_size.
  BlockSparseMatrix(std::vector<BlockTriplet>&& blocks,
                    std::vector<int>&& block_row_size,
                    std::vector<int>&& block_col_size);

  int rows_{0};                       // total number of rows.
  int cols_{0};                       // total number of columns.
  std::vector<BlockTriplet> blocks_;  // All block triplets.
  // block_row_size_[i] stores the number of rows in the i-th block row.
  std::vector<int> block_row_size_;
  // block_col_size_[j] stores the number of columns in the j-th block columns.
  std::vector<int> block_col_size_;
  // An ib-th row block will have dense entries i=from row_start_[ib] to
  // i=row_start_[ib]+block_row_size_[ib]-1.
  std::vector<int> row_start_;
  // A jb-th column block will have dense entries j=from col_start_[jb] to
  // j=col_start_[jb]+block_col_size_[jb]-1.
  std::vector<int> col_start_;
};

// Class used to build a BlockSparseMatrix.
// It provides safe APIs to add blocks that verify proper invariants as blocks
// are added. If successful, the resulting matrix obtained with the Build()
// method is properly formed.
// @tparam_nonsymbolic_scalar
template <typename T>
class BlockSparseMatrixBuilder {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(BlockSparseMatrixBuilder)

  // Instantiates a builder to make a BlockSparseMatrix with a known number of
  // blocks.
  // @param[in] block_rows Number of row blocks.
  // @param[in] block_cols Number of columns blocks.
  // @param[in] num_nonzero_blocks Total number of non-zero blocks.
  BlockSparseMatrixBuilder(int block_rows, int block_cols,
                           int num_nonzero_blocks)
      : block_rows_(block_rows), block_cols_(block_cols) {
    blocks_.reserve(num_nonzero_blocks);
    // Negative size means "not yet specified".
    block_row_size_.resize(block_rows, -1);
    block_col_size_.resize(block_cols, -1);
  }

  // Adds dense block Bij to the block entry with indexes (i,j).
  // @pre Bij.rows() must have the same number of rows as any previously added
  // block to row i or an exception is thrown.
  // @pre Bij.cols() must have the same number of columns as any previously
  // added block to column j or an exception is thrown.
  // @note Blocks of size zero are ignored.
  // @throws if block (i,j) was already added.
  void PushBlock(int i, int j, const MatrixX<T>& Bij);

  // Makes a new BlockSparseMatrix.
  // If successful, the new BlockSparseMatrix is guaranteed to be properly
  // formed. This builder is left in an invalid state. Do not reuse.
  BlockSparseMatrix<T> Build();

 private:
  void VerifyInvariants() const {
    // All rows/blocks must be specified in order to determine sizes. We verify
    // this is true and throw an exception if not.
    for (size_t ib = 0; ib < block_row_size_.size(); ++ib) {
      if (block_row_size_[ib] < 0) {
        throw std::runtime_error(
            fmt::format("No block was specified for row {:d}.", ib));
      }
    }
    for (size_t jb = 0; jb < block_col_size_.size(); ++jb) {
      if (block_col_size_[jb] < 0) {
        throw std::runtime_error(
            fmt::format("No block was specified for column {:d}.", jb));
      }
    }
  }

  int block_rows_{0};
  int block_cols_{0};
  std::vector<typename BlockSparseMatrix<T>::BlockTriplet> blocks_;
  std::vector<int> block_row_size_;
  std::vector<int> block_col_size_;
  // We store all pairs (i,j) for block Bij as the users adds blocks. This is
  // used to verify if a user already added a given block.
  struct pair_hash {
    template <class T1, class T2>
    std::size_t operator()(std::pair<T1, T2> const& pair) const {
      std::size_t h1 = std::hash<T1>()(pair.first);
      std::size_t h2 = std::hash<T2>()(pair.second);
      return h1 ^ h2;
    }
  };
  std::unordered_set<std::pair<int, int>, pair_hash> index_pairs_;
};

}  // namespace internal
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake
