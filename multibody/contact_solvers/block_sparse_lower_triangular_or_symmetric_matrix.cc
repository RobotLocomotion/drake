#include "drake/multibody/contact_solvers/block_sparse_lower_triangular_or_symmetric_matrix.h"

#include <algorithm>

#include <fmt/format.h>

#include "drake/common/drake_throw.h"
#include "drake/common/fmt_eigen.h"

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace internal {

int BlockSparsityPattern::CalcNumNonzeros() const {
  int result = 0;
  for (int i = 0; i < ssize(block_sizes_); ++i) {
    const std::vector<int>& neighbors_i = neighbors_[i];
    int row_count = 0;
    for (int n : neighbors_i) {
      row_count += block_sizes_[n];
    }
    result += row_count * block_sizes_[i];
  }
  return result;
}

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
void BlockSparseLowerTriangularOrSymmetricMatrix<MatrixType, is_symmetric>::
    MultiplySerial(const VectorX<double>& x,
                   EigenPtr<VectorX<double>> y) const {
  DRAKE_ASSERT(x.size() == cols());
  DRAKE_ASSERT(y->size() == rows());
  y->setZero();

  for (int j = 0; j < block_cols_; ++j) {
    const int col_start = starting_cols_[j];
    const int col_size = sparsity_pattern_.block_sizes()[j];
    // slice of x corresponding to block j
    auto x_j = x.segment(col_start, col_size);

    /* Loop over the stored lower-triangular blocks in column j. */
    for (int flat = 0; flat < ssize(blocks_[j]); ++flat) {
      int i = sparsity_pattern_.neighbors()[j][flat];  // block-row index
      const auto& A_ij = blocks_[j][flat];
      int row_start = starting_cols_[i];

      /* y_i += A_ij * x_j */
      y->segment(row_start, A_ij.rows()).noalias() += A_ij * x_j;

      /* if symmetric and off-diagonal, also do y_j += A_ijᵀ * x_i */
      if constexpr (is_symmetric) {
        if (i != j) {
          auto x_i = x.segment(row_start, A_ij.rows());
          y->segment(col_start, col_size).noalias() += A_ij.transpose() * x_i;
        }
      }
    }
  }
}

template <typename MatrixType, bool is_symmetric>
void BlockSparseLowerTriangularOrSymmetricMatrix<
    MatrixType, is_symmetric>::BuildRowNeighbors() const {
  /* Precompute, for each block-row i, the list of block-cols j≤i
   such that A(i,j) is stored in blocks_[j][…]. */
  row_neighbors_.resize(block_cols_);
  const auto& col_neighbors = sparsity_pattern_.neighbors();
  for (int j = 0; j < block_cols_; ++j) {
    for (int i : col_neighbors[j]) {
      /* neighbor lists only carry i>=j, so this appends exactly those j≤i */
      row_neighbors_[i].push_back(j);
    }
  }
}

template <typename MatrixType, bool is_symmetric>
void BlockSparseLowerTriangularOrSymmetricMatrix<
    MatrixType, is_symmetric>::MultiplyParallel(const VectorX<double>& x,
                                                EigenPtr<VectorX<double>> y,
                                                Parallelism parallelism) const {
  DRAKE_ASSERT(x.size() == cols());
  DRAKE_ASSERT(y->size() == rows());
  y->setZero();

  if (row_neighbors_.empty()) {
    BuildRowNeighbors();
  }

  /* alias for readability. */
  const auto& col_neighbors = sparsity_pattern_.neighbors();
  const auto& row_neighbors = row_neighbors_;
  const auto& sizes = sparsity_pattern_.block_sizes();
  const auto& starts = starting_cols_;
  const auto& book = block_row_to_flat_;
  const auto& blocks = blocks_;

  [[maybe_unused]] const int num_threads = parallelism.num_threads();
#if defined(_OPENMP)
#pragma omp parallel for num_threads(num_threads)
#endif
  for (int bi = 0; bi < block_cols_; ++bi) {
    /* y_block = y.segment(starts[bi], sizes[bi]) */
    const int r0 = starts[bi], rn = sizes[bi];
    auto y_block = y->segment(r0, rn);

    // Lower-triangular (j ≤ bi) and diagonal contributions:
    for (int j : row_neighbors[bi]) {
      int flat = book[j][bi];  // index into blocks_[j]
      const auto& M = blocks[j][flat];
      y_block.noalias() += M * x.segment(starts[j], sizes[j]);
    }

    if constexpr (is_symmetric) {
      // Upper-triangular (j > bi) via transposes.
      // col_neighbors[bi] lists all row-blocks i ≥ bi for column bi,
      // so skip the first (the diagonal) and flip the stored block.
      const auto& down_list = col_neighbors[bi];
      for (int k = 1; k < ssize(down_list); ++k) {
        int bj = down_list[k];    // bj > bi
        int flat = book[bi][bj];  // block (bj,bi) is at blocks_[bi][flat]
        const auto& M = blocks[bi][flat];
        y_block.noalias() += M.transpose() * x.segment(starts[bj], sizes[bj]);
      }
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
  return MakeDenseBottomRightCorner(block_cols());
}

template <typename MatrixType, bool is_symmetric>
MatrixX<double> BlockSparseLowerTriangularOrSymmetricMatrix<
    MatrixType, is_symmetric>::MakeDenseBottomRightCorner(const int num_blocks)
    const {
  DRAKE_DEMAND(0 <= num_blocks && num_blocks <= block_cols());
  if (num_blocks == 0) {
    return MatrixX<double>::Zero(0, 0);
  }
  const int block_col_start = block_cols() - num_blocks;
  /* The row/column in `this` matrix that corresponds to the 0,0-th entry in the
   dense result. */
  const int col_start = starting_cols_[block_col_start];
  const int row_start = col_start;
  MatrixX<double> result =
      MatrixX<double>::Zero(rows() - row_start, cols() - col_start);
  for (int j = block_col_start; j < block_cols(); ++j) {
    for (int flat = 0; flat < ssize(block_row_indices(j)); ++flat) {
      const int i = block_row_indices(j)[flat];
      const int num_rows = sparsity_pattern_.block_sizes()[i];
      const int num_cols = sparsity_pattern_.block_sizes()[j];
      /* The starting row and column indices of the block being copied in `this`
       matrix (the source). */
      const int src_row = starting_cols_[i];
      const int src_col = starting_cols_[j];
      /* The starting row and column indices of the block being copied in the
       resulting dense matrix (the destination). */
      const int dest_row = src_row - row_start;
      const int dest_col = src_col - col_start;
      DRAKE_DEMAND(dest_row >= 0);
      DRAKE_DEMAND(dest_col >= 0);
      result.block(dest_row, dest_col, num_rows, num_cols) = blocks_[j][flat];
      if (i != j && is_symmetric) {
        result.block(dest_col, dest_row, num_cols, num_rows) =
            blocks_[j][flat].transpose();
      }
    }
  }
  return result;
}

template <typename MatrixType, bool is_symmetric>
void BlockSparseLowerTriangularOrSymmetricMatrix<MatrixType, is_symmetric>::
    ZeroRowsAndColumns(const std::set<int>& indices) {
  DRAKE_THROW_UNLESS(is_symmetric);
  for (int j : indices) {
    if (!(0 <= j && j < block_cols())) {
      throw std::logic_error(fmt::format(
          "Input index out of range. Indices must lie in [0, {}); {} is given.",
          block_cols(), j));
    }
  }
  for (int j = 0; j < block_cols(); ++j) {
    /* Zero all blocks in the column except the diagonal if j is among the
     row/column indices to be zeroed out. */
    if (indices.contains(j)) {
      /* We want to keep the conditioning of the matrix in the ballpark of the
       original matrix (e.g. setting diagonal blocks to identity when the
       original values are the on order of millions is a bad idea), so we keep
       only the diagonal entries of the diagonal blocks and zero the
       off-diagonal entries. */
      /* Calling eval() here is important to avoid aliasing. */
      blocks_[j][0] = blocks_[j][0].diagonal().eval().asDiagonal();
      /* Zero all off diagonal blocks in the j-th column. */
      for (int flat = 1; flat < ssize(blocks_[j]); ++flat) {
        blocks_[j][flat].setZero();
      }
    } else {
      /* Otherwise, zero out all blocks with block row indices in the set. */
      for (int i : indices) {
        const int flat = block_row_to_flat_[j][i];
        if (flat >= 0) {
          blocks_[j][flat].setZero();
        }
      }
    }
  }
}

template <typename MatrixType, bool is_symmetric>
void BlockSparseLowerTriangularOrSymmetricMatrix<
    MatrixType, is_symmetric>::AssertValid(int i, int j,
                                           const std::optional<MatrixType>& Aij,
                                           const char* source) const {
  if (!(0 <= j && j <= i && i <= block_rows())) {
    throw std::runtime_error(fmt::format(
        "{}: block indices out of bound. It is required that 0 <= j && j <= i "
        "&& i < block_rows(). Instead, i = {}, j = {}, block_rows() = {}.",
        source, i, j, block_rows()));
  }
  if (!HasBlock(i, j)) {
    throw std::runtime_error(fmt::format(
        "{}: The requested {},{}-th block doesn't exist.", source, i, j));
  }

  auto is_block_symmetric = [](const MatrixType& M) {
    /* Note that "<=" is critical to allow zero matrices to pass. */
    return (M - M.transpose()).norm() <= 1e-12 * M.norm();
  };

  if (is_symmetric && i == j && Aij.has_value() && !is_block_symmetric(*Aij)) {
    throw std::runtime_error(
        fmt::format("{}: The {}-th diagonal block must be symmetric for a "
                    "symmetric matrix. Instead, the block is:\n {}",
                    source, i, fmt_eigen(*Aij)));
  }
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
