#include "drake/multibody/fixed_fem/dev/schur_complement.h"

namespace drake {
namespace multibody {
namespace fem {
namespace internal {

template <typename T>
SchurComplement<T>::SchurComplement(
    const Eigen::Ref<const Eigen::SparseMatrix<T>>& A,
    const Eigen::Ref<const Eigen::SparseMatrix<T>>& B_transpose,
    const Eigen::Ref<const Eigen::SparseMatrix<T>>& D)
    : p_(A.rows()), q_(D.rows()) {
  DRAKE_DEMAND(A.cols() == p_);
  DRAKE_DEMAND(D.cols() == q_);
  DRAKE_DEMAND(B_transpose.rows() == q_);
  DRAKE_DEMAND(B_transpose.cols() == p_);
  /* Special treatment for M = A is needed because Eigen::LLT::solve() throws
   exception if the matrix under decomposition is empty. */
  if (q_ == 0) {
    neg_Dinv_B_transpose_.resize(0, p_);
    D_complement_ = A;
  } else if (p_ == 0) {
    neg_Dinv_B_transpose_.resize(q_, 0);
    // D_complement is empty, same as default.
  } else {
    /* Note that since D is SPD, we could use a sparse Cholesky factorization
     here. But we opt for SparseLU for now due to the license restriction on
     Eigen's sparse Cholesky. */
    const Eigen::SparseLU<Eigen::SparseMatrix<T>> D_factorization(D);
    /* A column major rhs is required for the SparseLU solve, so we take
     `B_transpose` instead of `B.transpose()`. */
    neg_Dinv_B_transpose_ = D_factorization.solve(-B_transpose);
    D_complement_ = A + B_transpose.transpose() * neg_Dinv_B_transpose_;
  }
}

template <typename T>
VectorX<T> SchurComplement<T>::SolveForY(
    const Eigen::Ref<const VectorX<T>>& x) const {
  /* If M = D, then the system reduces to Dy = 0. */
  if (p_ == 0) {
    return VectorX<T>::Zero(q_);
  }
  DRAKE_DEMAND(x.size() == p_);
  return neg_Dinv_B_transpose_ * x;
}

}  // namespace internal
}  // namespace fem
}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::multibody::fem::internal::SchurComplement)
