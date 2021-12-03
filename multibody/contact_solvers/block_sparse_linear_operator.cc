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
  // TODO(amcastro-tri): avoid heap allocations for performance.
  const VectorX<T> x_dense = x;
  VectorX<T> y_dense(y->size());
  A_->Multiply(x_dense, &y_dense);
  *y = y_dense.sparseView();
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
  // TODO(amcastro-tri): avoid heap allocations for performance.
  const VectorX<T> x_dense = x;
  VectorX<T> y_dense(y->size());
  A_->MultiplyByTranspose(x_dense, &y_dense);
  *y = y_dense.sparseView();
}

template <typename T>
void BlockSparseLinearOperator<T>::DoAssembleMatrix(
    Eigen::SparseMatrix<T>* A_sparse) const {
  // Count number of non-zeros.
  int num_nonzeros = 0;
  for (const auto& [ib, jb, B] : A_->get_blocks()) {
    unused(ib, jb);
    num_nonzeros += B.size();
  }
  std::vector<Eigen::Triplet<T>> non_zeros;
  non_zeros.reserve(num_nonzeros);

  for (const auto& [ib, jb, B] : A_->get_blocks()) {
    for (int n = 0; n < B.cols(); ++n) {
      const int j = A_->col_start(jb) + n;
      for (int m = 0; m < B.rows(); ++m) {
        const int i = A_->row_start(ib) + m;
        non_zeros.push_back({i, j, B(m, n)});
      }
    }
  }
  A_sparse->setFromTriplets(non_zeros.begin(), non_zeros.end());
}

template <typename T>
void BlockSparseLinearOperator<T>::DoAssembleMatrix(
    BlockSparseMatrix<T>* A) const {
  *A = *A_;
}

}  // namespace internal
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::multibody::contact_solvers::internal::
        BlockSparseLinearOperator)
