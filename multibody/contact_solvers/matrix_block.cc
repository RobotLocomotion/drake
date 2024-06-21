#include "drake/multibody/contact_solvers/matrix_block.h"

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace internal {

template <class T>
MatrixBlock<T>::MatrixBlock(Block3x3SparseMatrix<T> data)
    : data_(std::move(data)), is_dense_(false) {}

template <class T>
MatrixBlock<T>::MatrixBlock(MatrixX<T> data)
    : data_(std::move(data)), is_dense_(true) {}

template <class T>
int MatrixBlock<T>::rows() const {
  return std::visit(
      [](auto&& arg) {
        /* We need the static_cast here because Eigen's rows() is `long`. */
        return static_cast<int>(arg.rows());
      },
      data_);
}

template <class T>
int MatrixBlock<T>::cols() const {
  return std::visit(
      [](auto&& arg) {
        /* We need the static_cast here because Eigen's cols() is `long`. */
        return static_cast<int>(arg.cols());
      },
      data_);
}

template <class T>
void MatrixBlock<T>::MultiplyAndAddTo(const Eigen::Ref<const MatrixX<T>>& A,
                                      EigenPtr<MatrixX<T>> y) const {
  DRAKE_DEMAND(y != nullptr);
  DRAKE_DEMAND(cols() == A.rows());
  DRAKE_DEMAND(rows() == y->rows());
  DRAKE_DEMAND(A.cols() == y->cols());

  if (is_dense_) {
    const MatrixX<T>& M_dense = std::get<MatrixX<T>>(data_);
    *y += M_dense * A;
    return;
  }
  const Block3x3SparseMatrix<T>& M_sparse =
      std::get<Block3x3SparseMatrix<T>>(data_);
  M_sparse.MultiplyAndAddTo(A, y);
}

template <class T>
void MatrixBlock<T>::TransposeAndMultiplyAndAddTo(
    const Eigen::Ref<const MatrixX<T>>& A, EigenPtr<MatrixX<T>> y) const {
  DRAKE_DEMAND(y != nullptr);
  DRAKE_DEMAND(cols() == y->rows());
  DRAKE_DEMAND(rows() == A.rows());
  DRAKE_DEMAND(A.cols() == y->cols());

  if (is_dense_) {
    const MatrixX<T>& M_dense = std::get<MatrixX<T>>(data_);
    *y += M_dense.transpose() * A;
    return;
  }
  const Block3x3SparseMatrix<T>& M_sparse =
      std::get<Block3x3SparseMatrix<T>>(data_);
  M_sparse.TransposeAndMultiplyAndAddTo(A, y);
}

// TODO(xuchenhan-tri): consider a double dispatch strategy where each block
// type provides an API to operate on every other block type.
template <class T>
void MatrixBlock<T>::TransposeAndMultiplyAndAddTo(
    const MatrixBlock<T>& A, EigenPtr<MatrixX<T>> y) const {
  DRAKE_DEMAND(y != nullptr);
  DRAKE_DEMAND(cols() == y->rows());
  DRAKE_DEMAND(rows() == A.rows());
  DRAKE_DEMAND(A.cols() == y->cols());
  if (A.is_dense_) {
    const MatrixX<T>& A_dense = std::get<MatrixX<T>>(A.data_);
    this->TransposeAndMultiplyAndAddTo(A_dense, y);
    return;
  }

  /* A is sparse. */
  const Block3x3SparseMatrix<T>& A_sparse =
      std::get<Block3x3SparseMatrix<T>>(A.data_);
  if (this->is_dense_) {
    const MatrixX<T>& M_dense = std::get<MatrixX<T>>(this->data_);
    A_sparse.LeftMultiplyAndAddTo(M_dense.transpose(), y);
    return;
  }

  /* A and M are both sparse. */
  const Block3x3SparseMatrix<T>& M_sparse =
      std::get<Block3x3SparseMatrix<T>>(this->data_);
  M_sparse.TransposeAndMultiplyAndAddTo(A_sparse, y);
}

// TODO(xuchenhan-tri): consider a double dispatch strategy where each block
// type provides an API to operate on every other block type.
template <class T>
MatrixBlock<T> MatrixBlock<T>::LeftMultiplyByBlockDiagonal(
    const std::vector<MatrixX<T>>& Gs, int start, int end) const {
  DRAKE_DEMAND(start >= 0);
  DRAKE_DEMAND(end >= start);
  DRAKE_DEMAND(static_cast<int>(Gs.size()) > end);
  /* Verify that the sizes of G and M is compatible. */
  std::vector<int>
      row_starts;  // starting row index for each diagonal block in G
  row_starts.reserve(end - start + 1);
  int row = 0;
  for (int i = start; i <= end; ++i) {
    DRAKE_DEMAND(Gs[i].rows() == Gs[i].cols());
    row_starts.emplace_back(row);
    row += Gs[i].rows();
    if (!is_dense_) {
      DRAKE_DEMAND(Gs[i].rows() % 3 == 0);
    }
  }
  DRAKE_DEMAND(row == rows());

  if (is_dense_) {
    const MatrixX<T>& M_dense = std::get<MatrixX<T>>(data_);
    MatrixX<T> GM(rows(), cols());
    for (int i = start; i <= end; ++i) {
      const int rows = Gs[i].rows();
      const int row_start = row_starts[i - start];
      GM.middleRows(row_start, rows).noalias() =
          Gs[i] * M_dense.middleRows(row_start, rows);
    }
    return MatrixBlock<T>(std::move(GM));
  }

  const Block3x3SparseMatrix<T>& M_sparse =
      std::get<Block3x3SparseMatrix<T>>(data_);
  /* At this point, we know that M is block 3x3 sparse and G is block diagonal,
   G is block diagonal, with block h being 3nₕ x 3nₕ for positive integers nₕ.
   Therefore, all quantities in the computation of A = G * M can be represented
   as 3x3 blocks. We loop over all 3x3 non-zero blocks of M, and for each Mₖⱼ
   (the k,j-th 3x3 block of M), we find all non-zero 3x3 blocks Gᵢₖ that
   multiplies with Mₖⱼ, and add their product to the resulting 3x3 block, Aᵢⱼ.

   Because G is block diagonal, to achieve the above, we only need:
   (1) The index h into `Gs` to obtain the correct diagonal block of G that
       multiplies with Mₖⱼ.
   (2) The block row indices (i.e. the i's) of these non-zero 3x3 G subblocks.

   Note that, for (2), the block row indices are contiguous, so we only need the
   starting block row index and the size of that G block to determine all the
   block row indices. */
  std::vector<int> G_indices(
      M_sparse.block_rows());  // `G_indices[k]` gives the relevant index h into
                               // `Gs` for block column k.
  std::vector<int> i_start(
      M_sparse.block_rows());  // `i_start[k]` gives the starting block row
                               // index from (2) above.
  {
    int i = 0;
    int h = start;
    for (int k = 0; k < M_sparse.block_rows(); ++k) {
      if (3 * k >= row_starts[h - start] + Gs[h].rows()) {
        i += Gs[h].cols() / 3;
        ++h;
      }
      i_start[k] = i;
      G_indices[k] = h;
    }
  }
  /* We also record, for each k, the local block column index, l, such that the
   `Gs[G_indices[k]].col(l)` gives the (non-zero entries of) k-th column of G.
   */
  std::vector<int> local_block_cols(M_sparse.block_rows());
  for (int k = 0; k < M_sparse.block_rows(); ++k) {
    local_block_cols[k] = k - i_start[k];
  }

  Block3x3SparseMatrix<T> A(M_sparse.block_rows(), M_sparse.block_cols());
  using Triplet = typename Block3x3SparseMatrix<T>::Triplet;
  const std::vector<std::vector<Triplet>>& M_triplets = M_sparse.get_triplets();
  std::vector<Triplet> A_triplets;
  A_triplets.reserve(M_sparse.num_blocks());
  for (const auto& row_data : M_triplets) {
    for (const Triplet& t : row_data) {
      const int k = std::get<0>(t);
      const int j = std::get<1>(t);
      const Matrix3<T>& M_block = std::get<2>(t);
      const int h = G_indices[k];
      /* Given k and j, perform Aᵢⱼ+= Gᵢₖ * Mₖⱼ for all relevant i's. */
      for (int l = 0; 3 * l < Gs[h].cols(); ++l) {
        const int i = i_start[k] + l;
        A_triplets.emplace_back(
            i, j,
            Gs[h].template block<3, 3>(3 * l, 3 * local_block_cols[k]) *
                M_block);
      }
    }
  }
  A.SetFromTriplets(A_triplets);
  return MatrixBlock<T>(std::move(A));
}

template <class T>
void MatrixBlock<T>::MultiplyWithScaledTransposeAndAddTo(
    const VectorX<T>& scale, EigenPtr<MatrixX<T>> y) const {
  DRAKE_DEMAND(y != nullptr);
  DRAKE_DEMAND(cols() == scale.size());
  DRAKE_DEMAND(rows() == y->rows());
  DRAKE_DEMAND(rows() == y->cols());

  if (is_dense_) {
    const MatrixX<T>& M_dense = std::get<MatrixX<T>>(data_);
    *y += M_dense * scale.asDiagonal() * M_dense.transpose();
    return;
  }
  const Block3x3SparseMatrix<T>& M_sparse =
      std::get<Block3x3SparseMatrix<T>>(data_);
  M_sparse.MultiplyWithScaledTransposeAndAddTo(scale, y);
}

template <class T>
MatrixX<T> MatrixBlock<T>::MakeDenseMatrix() const {
  if (is_dense_) {
    return std::get<MatrixX<T>>(data_);
  }
  return std::get<Block3x3SparseMatrix<T>>(data_).MakeDenseMatrix();
}

template <typename T>
MatrixBlock<T> StackMatrixBlocks(const std::vector<MatrixBlock<T>>& blocks) {
  if (blocks.empty()) {
    return {};
  }

  const bool is_dense = blocks[0].is_dense_;
  const int cols = blocks[0].cols();
  int rows = 0;
  for (const auto& b : blocks) {
    DRAKE_DEMAND(is_dense == b.is_dense_);
    DRAKE_DEMAND(cols == b.cols());
    rows += b.rows();
  }

  if (is_dense) {
    MatrixX<T> result(rows, cols);
    int row_offset = 0;
    for (const auto& b : blocks) {
      result.middleRows(row_offset, b.rows()) = std::get<MatrixX<T>>(b.data_);
      row_offset += b.rows();
    }
    return MatrixBlock<T>(std::move(result));
  }

  /* Each entry in `blocks` is Block3x3SparseMatrix. */
  DRAKE_DEMAND(rows % 3 == 0);
  DRAKE_DEMAND(cols % 3 == 0);
  const int block_rows = rows / 3;
  const int block_cols = cols / 3;
  int block_row_offset = 0;
  Block3x3SparseMatrix<T> result(block_rows, block_cols);
  using Triplet = typename Block3x3SparseMatrix<T>::Triplet;
  std::vector<Triplet> result_triplets;
  int nonzero_blocks = 0;
  for (const auto& b : blocks) {
    const Block3x3SparseMatrix<T>& entry =
        std::get<Block3x3SparseMatrix<T>>(b.data_);
    nonzero_blocks += entry.num_blocks();
  }
  result_triplets.reserve(nonzero_blocks);

  for (const auto& b : blocks) {
    const Block3x3SparseMatrix<T>& entry =
        std::get<Block3x3SparseMatrix<T>>(b.data_);
    const std::vector<std::vector<Triplet>>& b_triplets = entry.get_triplets();
    for (const auto& row_data : b_triplets) {
      for (const Triplet& t : row_data) {
        const int block_row = std::get<0>(t) + block_row_offset;
        const int block_col = std::get<1>(t);
        const Matrix3<T>& m = std::get<2>(t);
        result_triplets.emplace_back(block_row, block_col, m);
      }
    }
    block_row_offset += entry.block_rows();
  }
  result.SetFromTriplets(result_triplets);
  return MatrixBlock<T>(std::move(result));
}

DRAKE_DEFINE_FUNCTION_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    (&StackMatrixBlocks<T>));

}  // namespace internal
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::multibody::contact_solvers::internal::MatrixBlock);
