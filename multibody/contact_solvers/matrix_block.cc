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
                                      EigenPtr<VectorX<T>> y) const {
  DRAKE_DEMAND(y != nullptr);
  DRAKE_DEMAND(cols() == A.rows());
  DRAKE_DEMAND(rows() == y->size());

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

template <class T>
void MatrixBlock<T>::TransposeAndMultiplyAndAddTo(const MatrixBlock<T>& A,
                                                 EigenPtr<MatrixX<T>> y) const {
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

template <class T>
MatrixBlock<T> MatrixBlock<T>::LeftMultiplyByBlockDiagonal(
    const std::vector<MatrixX<T>>& Gs, int start, int end) const {
  DRAKE_DEMAND(start >= 0);
  DRAKE_DEMAND(end >= start);
  DRAKE_DEMAND(static_cast<int>(Gs.size()) > end);
  /* Verify that the sizes of G and M is compatible. */
  int G_rows = 0;
  for (int i = start; i <= end; ++i) {
    DRAKE_DEMAND(Gs[i].rows() == Gs[i].cols());
    G_rows += Gs[i].rows();
    if (!is_dense_) {
      DRAKE_DEMAND(Gs[i].rows() == 3);
    }
  }
  DRAKE_DEMAND(G_rows == rows());

  if (is_dense_) {
    const MatrixX<T>& M_dense = std::get<MatrixX<T>>(data_);
    MatrixX<T> GM(rows(), cols());
    int row_offset = 0;
    for (int i = start; i <= end; ++i) {
      const int rows = Gs[i].rows();
      GM.middleRows(row_offset, rows).noalias() =
          Gs[i] * M_dense.middleRows(row_offset, rows);
      row_offset += rows;
    }
    return MatrixBlock<T>(std::move(GM));
  }

  const Block3x3SparseMatrix<T>& M_sparse =
      std::get<Block3x3SparseMatrix<T>>(data_);
  Block3x3SparseMatrix<T> GM(M_sparse.block_rows(), M_sparse.block_cols());
  using Triplet = typename Block3x3SparseMatrix<T>::Triplet;
  const std::vector<std::vector<Triplet>>& M_triplets = M_sparse.get_triplets();
  std::vector<Triplet> GM_triplets;
  GM_triplets.reserve(M_sparse.num_blocks());
  for (const auto& row_data : M_triplets) {
    for (const Triplet& t : row_data) {
      const int block_row = std::get<0>(t);
      const int block_col = std::get<1>(t);
      const Matrix3<T>& M_block = std::get<2>(t);
      GM_triplets.emplace_back(block_row, block_col,
                               Gs[start + block_row] * M_block);
    }
  }
  GM.SetFromTriplets(GM_triplets);
  return MatrixBlock<T>(std::move(GM));
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
    (&StackMatrixBlocks<T>))

}  // namespace internal
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::multibody::contact_solvers::internal::MatrixBlock)
