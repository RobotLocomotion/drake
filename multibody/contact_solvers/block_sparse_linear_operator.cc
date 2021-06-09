#include "drake/multibody/contact_solvers/block_sparse_linear_operator.h"

#include <vector>

#include "drake/common/unused.h"

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace internal {

template <typename T>
BlockSparseLinearOperator<T>::BlockSparseLinearOperator(
    const std::string& name, const BlockSparseMatrix<T>* A)
    : LinearOperator<T>(name), A_(A) {
  DRAKE_DEMAND(A != nullptr);
  row_vec_.resize(A->rows());
  col_vec_.resize(A->cols());
}

template <typename T>
void BlockSparseLinearOperator<T>::DoMultiply(
    const Eigen::Ref<const VectorX<T>>& x, VectorX<T>* y) const {
  A_->Multiply(x, y);
}

template <typename T>
void BlockSparseLinearOperator<T>::DoMultiply(
    const Eigen::Ref<const Eigen::SparseVector<T>>& x,
    Eigen::SparseVector<T>* y) const {
  // TODO(amcastro-tri): consider alternative to avoid copies from/to sparse
  // vector.
  col_vec_ = x;
  A_->Multiply(col_vec_, &row_vec_);
  *y = row_vec_.sparseView();
}

template <typename T>
void BlockSparseLinearOperator<T>::DoMultiplyByTranspose(
    const Eigen::Ref<const VectorX<T>>& x, VectorX<T>* y) const {
  A_->MultiplyByTranspose(x, y);
}

template <typename T>
void BlockSparseLinearOperator<T>::DoMultiplyByTranspose(
    const Eigen::Ref<const Eigen::SparseVector<T>>& x,
    Eigen::SparseVector<T>* y) const {
  // TODO(amcastro-tri): consider alternative to avoid copies from/to sparse
  // vector.
  row_vec_ = x;
  A_->MultiplyByTranspose(row_vec_, &col_vec_);
  *y = col_vec_.sparseView();
}

template <typename T>
void BlockSparseLinearOperator<T>::DoAssembleMatrix(
    Eigen::SparseMatrix<T>* A) const {
  // Count number of non-zeros.
  int num_nnz = 0;
  for (const auto& [ib, jb, B] : A_->get_blocks()) {
    unused(ib);
    unused(jb);
    num_nnz += B.size();
  }
  std::vector<Eigen::Triplet<T>> non_zeros;
  non_zeros.reserve(num_nnz);

  for (const auto& [ib, jb, B] : A_->get_blocks()) {
    for (int n = 0; n < B.cols(); ++n) {
      const int j = A_->col_start(jb) + n;
      for (int m = 0; m < B.rows(); ++m) {
        const int i = A_->row_start(ib) + m;
        non_zeros.push_back({i, j, B(m, n)});
      }
    }
  }
  A->resize(A_->rows(), A_->cols());
  A->setFromTriplets(non_zeros.begin(), non_zeros.end());
}

}  // namespace internal
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::multibody::contact_solvers::internal::
        BlockSparseLinearOperator)
