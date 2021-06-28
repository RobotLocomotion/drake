#include "drake/multibody/fixed_fem/dev/schur_complement.h"

namespace drake {
namespace multibody {
namespace fixed_fem {
namespace internal {

template <typename T>
SchurComplement<T>::SchurComplement(const Eigen::Ref<const MatrixX<T>>& A,
                                    const Eigen::Ref<const MatrixX<T>>& B,
                                    const Eigen::Ref<const MatrixX<T>>& D)
    : p_(A.rows()), q_(D.rows()) {
  DRAKE_DEMAND(A.cols() == p_);
  DRAKE_DEMAND(D.cols() == q_);
  DRAKE_DEMAND(B.rows() == p_);
  DRAKE_DEMAND(B.cols() == q_);
  /* Special treatment for M = A is needed because Eigen::LLT::solve() throws
   exception if the matrix under decomposition is empty. */
  if (q_ == 0) {
    neg_Dinv_B_transpose_ = MatrixX<T>::Zero(0, 0);
    D_complement_ = A;
  } else {
    const Eigen::LLT<MatrixX<T>> D_factorization(D);
    neg_Dinv_B_transpose_ = D_factorization.solve(-B.transpose());
    D_complement_ = A + B * neg_Dinv_B_transpose_;
  }
}

template <typename T>
VectorX<T> SchurComplement<T>::SolveForY(const VectorX<T>& x) const {
  /* If M = D, then the system reduces to Dy = 0. */
  if (p_ == 0) {
    return VectorX<T>::Zero(q_);
  }
  DRAKE_DEMAND(x.size() == p_);
  return neg_Dinv_B_transpose_ * x;
}

}  // namespace internal
}  // namespace fixed_fem
}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::multibody::fixed_fem::internal::SchurComplement)
