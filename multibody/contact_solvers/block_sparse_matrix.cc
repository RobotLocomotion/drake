#include "drake/multibody/contact_solvers/block_sparse_matrix.h"

#include <tuple>
#include <vector>

#include "drake/common/default_scalars.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/common/unused.h"

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace internal {

template <typename T>
BlockSparseMatrix<T>::BlockSparseMatrix(std::vector<BlockTriplet>&& blocks,
                                        std::vector<int>&& block_row_size,
                                        std::vector<int>&& block_col_size)
    : blocks_(std::move(blocks)),
      block_row_size_(std::move(block_row_size)),
      block_col_size_(std::move(block_col_size)) {
  row_start_.resize(block_row_size_.size(), 0);
  col_start_.resize(block_col_size_.size(), 0);
  rows_ = 0;
  for (size_t ib = 0; ib < block_row_size_.size(); ++ib) {
    row_start_[ib] = rows_;
    // N.B. This assumes there are no empty rows.
    rows_ += block_row_size_[ib];
  }
  cols_ = 0;
  for (size_t jb = 0; jb < block_col_size_.size(); ++jb) {
    col_start_[jb] = cols_;
    // N.B. This assumes there are no empty columns.
    cols_ += block_col_size_[jb];
  }
}

template <typename T>
void BlockSparseMatrix<T>::Multiply(const Eigen::Ref<const VectorX<T>>& x,
                                    EigenPtr<VectorX<T>> y) const {
  DRAKE_DEMAND(x.size() == cols());
  DRAKE_DEMAND(y != nullptr);
  DRAKE_DEMAND(y->size() == rows());
  y->setZero();
  for (const auto& [ib, jb, Bij] : blocks_) {
    const auto xj = x.segment(col_start_[jb], block_col_size_[jb]);
    auto yi = y->segment(row_start_[ib], block_row_size_[ib]);
    // N.B. noalias() is necessary to remove Eigen's aliasing assumption and
    // avoid evaluation into a temporary.
    yi.noalias() += Bij * xj;
  }
}

template <typename T>
void BlockSparseMatrix<T>::MultiplyByTranspose(
    const Eigen::Ref<const VectorX<T>>& x, EigenPtr<VectorX<T>> y) const {
  DRAKE_DEMAND(x.size() == rows());
  DRAKE_DEMAND(y != nullptr);
  DRAKE_DEMAND(y->size() == cols());
  y->setZero();
  for (const auto& [ib, jb, Bij] : blocks_) {
    const auto xi = x.segment(row_start_[ib], block_row_size_[ib]);
    auto yj = y->segment(col_start_[jb], block_col_size_[jb]);
    // N.B. noalias() is necessary to remove Eigen's aliasing assumption and
    // avoid evaluation into a temporary.
    yj.noalias() += Bij.transpose() * xi;
  }
}

template <typename T>
MatrixX<T> BlockSparseMatrix<T>::MakeDenseMatrix() const {
  MatrixX<T> A(rows(), cols());
  A.setZero();
  for (const auto& [ib, jb, Bij] : blocks_) {
    const int is = row_start_[ib];
    const int rows = block_row_size_[ib];
    const int js = col_start_[jb];
    const int cols = block_col_size_[jb];
    A.block(is, js, rows, cols) = Bij;
  }
  return A;
}

template <typename T>
void BlockSparseMatrixBuilder<T>::PushBlock(int i, int j,
                                            const MatrixX<T>& Bij) {
  if (blocks_.size() == blocks_.capacity()) {
    throw std::runtime_error(
        "Exceeded the maximum number of non-zero blocks capacity specified at "
        "construction.");
  }
  if (Bij.size() > 0) {
    if (block_row_size_[i] >= 0) {
      // A block was added before to this row. Verify that Bij.rows() ==
      // block_row_size_[i].
      DRAKE_THROW_UNLESS(Bij.rows() == block_row_size_[i]);
    }
    if (block_col_size_[j] >= 0) {
      // A block was added before to this column. Verify that Bij.cols() ==
      // block_col_size_[j].
      DRAKE_THROW_UNLESS(Bij.cols() == block_col_size_[j]);
    }
    // Verify that the block was not already added.
    const auto& [it, success] = index_pairs_.emplace(i, j);
    unused(it);
    if (!success) {
      throw std::runtime_error(
          fmt::format("Block ({}, {}) already added.", i, j));
    }
    blocks_.push_back({i, j, Bij});
    block_row_size_[i] = Bij.rows();
    block_col_size_[j] = Bij.cols();
  }
}

template <typename T>
BlockSparseMatrix<T> BlockSparseMatrixBuilder<T>::Build() {
  VerifyInvariants();
  return BlockSparseMatrix<T>(std::move(blocks_), std::move(block_row_size_),
                              std::move(block_col_size_));
}

template <typename T>
void BlockSparseMatrixBuilder<T>::VerifyInvariants() const {
  // All row/column blocks must be specified in order to determine sizes. We
  // verify this is true and throw an exception if not.
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

}  // namespace internal
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::multibody::contact_solvers::internal::BlockSparseMatrix)

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::multibody::contact_solvers::internal::
        BlockSparseMatrixBuilder)
