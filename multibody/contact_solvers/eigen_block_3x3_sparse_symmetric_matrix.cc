#include "drake/multibody/contact_solvers/eigen_block_3x3_sparse_symmetric_matrix.h"

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace internal {

EigenBlock3x3SparseSymmetricMatrix::EigenBlock3x3SparseSymmetricMatrix(
    const Block3x3SparseSymmetricMatrix* A, Parallelism parallelism)
    : A_(A), parallelism_(parallelism) {
  /* Precompute, for each block-row i, the list of block-cols j≤i
   such that A(i,j) is stored in blocks[j][…]. */
  row_neighbors_.resize(A->block_cols());
  const auto& col_neighbors = A->sparsity_pattern().neighbors();
  for (int j = 0; j < A->block_cols(); ++j) {
    for (int i : col_neighbors[j]) {
      /* neighbor lists only carry i>=j, so this appends exactly those j≤i */
      row_neighbors_[i].push_back(j);
    }
  }
  /* Precompute the diagonal of the matrix A. */
  diagonal_.resize(A->cols());
  for (int j = 0; j < A_->block_cols(); ++j) {
    const Eigen::Matrix3d& block = A_->diagonal_block(j);
    diagonal_.segment<3>(3 * j) = block.diagonal();
  }
}

/* Performs y = A*x where A is this matrix.
 @pre x and y have sizes compatible with this matrix. */
void EigenBlock3x3SparseSymmetricMatrix::Multiply(const VectorX<double>& x,
                                                  VectorX<double>* y) const {
  DRAKE_ASSERT(y != nullptr);
  DRAKE_ASSERT(x.size() == cols());
  DRAKE_ASSERT(y->size() == rows());
  y->setZero();

  const int block_cols = A_->block_cols();
  const std::vector<std::vector<int>>& col_neighbors =
      A_->sparsity_pattern().neighbors();
  const std::vector<std::vector<int>>& row_neighbors = row_neighbors_;
  const std::vector<int>& starts = A_->starting_cols();
  const std::vector<std::vector<Matrix3<double>>>& blocks = A_->blocks();
  const std::vector<std::vector<int>>& block_row_to_flat =
      A_->block_row_to_flat();

  [[maybe_unused]] const int num_threads = parallelism_.num_threads();
#if defined(_OPENMP)
#pragma omp parallel for num_threads(num_threads)
#endif
  for (int bi = 0; bi < block_cols; ++bi) {
    auto y_block = y->segment<3>(starts[bi]);
    /* Lower-triangular (bj ≤ bi) and diagonal contributions: */
    for (int bj : row_neighbors[bi]) {
      const int flat = block_row_to_flat[bj][bi];  // index into blocks[bj]
      const Matrix3<double>& M = blocks[bj][flat];
      y_block.noalias() += M * x.segment<3>(starts[bj]);
    }
    /* Upper-triangular contributions (bj > bi) via transposes.
     col_neighbors[bi] lists all row-blocks i ≥ bi for column bi,
     so skip the first (the diagonal) and flip the stored block. */
    const std::vector<int>& down_list = col_neighbors[bi];
    for (int k = 1; k < ssize(down_list); ++k) {
      const int bj = down_list[k];  // bj > bi
      const Matrix3<double>& M = blocks[bi][k];
      y_block.noalias() += M.transpose() * x.segment<3>(starts[bj]);
    }
  }
}

}  // namespace internal
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake
