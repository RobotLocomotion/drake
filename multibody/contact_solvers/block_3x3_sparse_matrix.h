#pragma once

#include <tuple>
#include <vector>

#include "drake/common/default_scalars.h"
#include "drake/common/eigen_types.h"

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace internal {

/* A sparse matrix data structure composed of m-by-n block submatrices, where
 each submatrix is of size 3-by-3. Only the non-zero submatrices are stored for
 efficiency. We use `M` to denote `this` Block3x3SparseMatrix throughout this
 class.
 @tparam_default_scalar */
template <class T>
class Block3x3SparseMatrix {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(Block3x3SparseMatrix);

  /* A triplet is a tuple (i, j, value) defining a non-zero submatrix where i is
   the block row index, j is the block column index, and value is a 3x3 dense
   matrix. */
  using Triplet = std::tuple<int, int, Matrix3<T>>;

  /* Creates a Block3x3SparseMatrix with capacity for
   `block_rows`-by-`block_cols` block submatrices. The matrix is initialized to
   be empty, i.e. no non-zero blocks. */
  Block3x3SparseMatrix(int block_rows, int block_cols)
      : data_(block_rows),
        block_rows_(block_rows),
        block_cols_(block_cols),
        col_to_indices_(block_cols) {
    DRAKE_DEMAND(block_rows >= 0);
    DRAKE_DEMAND(block_cols >= 0);
  }

  /* Fills `this` matrix with the given vector of triplets. The input vector of
   triplets does not have to be sorted, but cannot contain duplicated items
   (i.e. triplet with the same block row and column index). The contents in
   `this` matrix prior to the call to this function is destroyed.
   @pre The block row and column indices for each triplet in the given vector is
   within [0, block_rows) and [0, column_blocks) prescribed in the constructor
   of this class.
   @warn `triplets` must not contain items with duplicated block row and column
   indices. */
  void SetFromTriplets(const std::vector<Triplet>& triplets);

  int rows() const { return block_rows_ * 3; }
  int cols() const { return block_cols_ * 3; }
  int block_rows() const { return block_rows_; }
  int block_cols() const { return block_cols_; }

  /* Performs *y += M * A.
   @pre y != nullptr and the sizes of A and y are compatible with M. */
  void MultiplyAndAddTo(const Eigen::Ref<const MatrixX<T>>& A,
                        EigenPtr<VectorX<T>> y) const;

  /* Performs *y += A * M.
   @pre y != nullptr and the sizes of A and y are compatible with M. */
  void LeftMultiplyAndAddTo(const Eigen::Ref<const MatrixX<T>>& A,
                            EigenPtr<MatrixX<T>> y) const;

  /* Performs *y += Mᵀ * A, where A is dense.
   @pre y != nullptr and the sizes of A and y are compatible with M. */
  void TransposeAndMultiplyAndAddTo(const Eigen::Ref<const MatrixX<T>>& A,
                                   EigenPtr<MatrixX<T>> y) const;

  /* Performs *y += Mᵀ * A, where A is also 3x3 block sparse.
   @pre y != nullptr and the sizes of A and y are compatible with M. */
  void TransposeAndMultiplyAndAddTo(const Block3x3SparseMatrix<T>& A,
                                   EigenPtr<MatrixX<T>> y) const;

  /* Performs *y += M * scale.asDiagonal() * M.transpose().
   @pre y != nullptr and the sizes of scale and y are compatible with M. */
  void MultiplyWithScaledTransposeAndAddTo(const VectorX<T>& scale,
                                         EigenPtr<MatrixX<T>> y) const;

  /* Returns the non-zero blocks in the matrix in a row-major fashion. Within
   each block row (result[row]), the blocks are sorted in increasing column
   indices. If result[row] is empty, then it means that there's no non-zero
   block in that block row. */
  const std::vector<std::vector<Triplet>>& get_triplets() const {
    return data_;
  }

  /* Returns the number of non-zero blocks. */
  int num_blocks() const { return num_blocks_; }

  /* Returns the matrix as an Eigen dense matrix. Useful for debugging. */
  MatrixX<T> MakeDenseMatrix() const;

 private:
  using RowData = std::vector<Triplet>;
  /* We store the non-zero blocks in the matrix in a row-major fashion. Within
   each row, the blocks are sorted in increasing column indices. If data_[i] is
   empty, then it means that there's no non-zero block in that block row. */
  std::vector<RowData> data_;
  int block_rows_{};
  int block_cols_{};
  int num_blocks_{};

  /* Index into `data_`. For a given `index`, data_[index.row][index.flat]
   retrieves the corresponding triplet. */
  struct Index {
    int row;
    int flat;
  };
  /* For each block column j, col_to_indices_[j] contains all the indices
   corresponding to non-zero entries in block column j in increasing row index.
   That is, col_to_indices_[j][i1].row < col_to_indices_[j][i2].row if i1 < i2.
   The same information can be computed from `data_`, but we store this
   redundant, precomputed information for convenience. */
  std::vector<std::vector<Index>> col_to_indices_;
};

}  // namespace internal
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake
