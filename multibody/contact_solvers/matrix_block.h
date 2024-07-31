#pragma once

#include <utility>
#include <variant>
#include <vector>

#include "drake/common/eigen_types.h"
#include "drake/multibody/contact_solvers/block_3x3_sparse_matrix.h"

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace internal {

template <typename T>
class MatrixBlock;

/* Row-wise concatenates the given vector of MatrixBlocks into a single
 MatrixBlock. In other words, given blocks [b₁, b₂, ..., bₙ], the resulting
 stacked matrix M is
         _    _
        |  b₁  |
        |  b₂  |
    M = |  ... |
        |  ... |
        |_ bₙ _|

 @pre all entries in `blocks` have the same `cols()`.
 @pre either all entries in `blocks` are dense, or all entries in `blocks` are
 sparse.
 @returns A MatrixBlock concatenating the given blocks row-wise that's dense if
 all the given blocks are dense and sparse otherwise.
 @tparam_default_scalar */
template <typename T>
MatrixBlock<T> StackMatrixBlocks(const std::vector<MatrixBlock<T>>& blocks);

/* Data structure to store individual non-zero blocks of BlockSparseMatrix.
 The MatrixBlock can either be dense or sparse, which is determined by how it is
 constructed. Right now, the only sparsity pattern supported is block 3-by-3
 (see Block3x3SparseMatrix). Once constructed, the MatrixBlock cannot change
 from a dense representation to a sparse one or vice versa. This class provides
 a series of matrix computation functions that exploits the sparsity if the
 MatrixBlock is sparse and falls back to dense algebra otherwise. We use `M` to
 denote `this` MatrixBlock throughout this class.
 @tparam_default_scalar */
template <class T>
class MatrixBlock {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(MatrixBlock);

  /* Constructs an empty MatrixBlock of size 0-by-0. */
  MatrixBlock() : MatrixBlock(MatrixX<T>::Zero(0, 0)) {}

  /* Constructs a MatrixBlock with the given Block3x3SparseMatrix. */
  explicit MatrixBlock(Block3x3SparseMatrix<T> data);

  /* Constructs a MatrixBlock with the given Eigen dense matrix. */
  explicit MatrixBlock(MatrixX<T> data);

  int rows() const;
  int cols() const;
  int size() const { return rows() * cols(); }

  bool is_dense() const { return is_dense_; }

  /* Performs *y += M * A.
   @pre y != nullptr and the sizes of A and y are compatible with M. */
  void MultiplyAndAddTo(const Eigen::Ref<const MatrixX<T>>& A,
                        EigenPtr<MatrixX<T>> y) const;

  /* Performs y += Mᵀ * A.
   @pre y != nullptr and the sizes of A and y are compatible with M. */
  void TransposeAndMultiplyAndAddTo(const Eigen::Ref<const MatrixX<T>>& A,
                                    EigenPtr<MatrixX<T>> y) const;

  // TODO(xuchenhan-tri): Consider writing the output to a MatrixBlock to
  // preserve the sparsity structure if it exists.
  /* Performs y += Mᵀ * A.
   @pre y != nullptr and the sizes of A and y are compatible with M. */
  void TransposeAndMultiplyAndAddTo(const MatrixBlock<T>& A,
                                    EigenPtr<MatrixX<T>> y) const;

  /* Computes G * M where G is a block diagonal matrix with the diagonal blocks
   specified as a vector of dense matrices. In particular, the diagonal blocks
   are [Gs[start], ..., Gs[end]]. The result is returned as a MatrixBlock,
   sparse if M is sparse, dense otherwise.
   @pre 0 <= start <= end < Gs.size().
   @pre All matrices in Gs are square.
   @pre Gs[start].rows() + ... + Gs[end].rows() == M.rows().
   @pre If M is sparse, all matrices in Gs are 3n-by-3n for some positive
   integer n. */
  MatrixBlock<T> LeftMultiplyByBlockDiagonal(const std::vector<MatrixX<T>>& Gs,
                                             int start, int end) const;

  /* Performs y += M * scale.asDiagonal() * M.transpose().
   @pre y != nullptr and the sizes of scale and y are compatible with M. */
  void MultiplyWithScaledTransposeAndAddTo(const VectorX<T>& scale,
                                           EigenPtr<MatrixX<T>> y) const;

  /* Returns the MatrixBlock as an Eigen dense matrix. Useful for debugging and
   testing. */
  MatrixX<T> MakeDenseMatrix() const;

  bool operator==(const MatrixBlock<T>&) const = default;

 private:
  friend MatrixBlock<T> StackMatrixBlocks<T>(
      const std::vector<MatrixBlock<T>>& blocks);

  std::variant<MatrixX<T>, Block3x3SparseMatrix<T>> data_;
  bool is_dense_{};
};

}  // namespace internal
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake
