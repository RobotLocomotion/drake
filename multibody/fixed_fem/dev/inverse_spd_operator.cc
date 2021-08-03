#include "drake/multibody/fixed_fem/dev/inverse_spd_operator.h"

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace internal {

template <typename T>
InverseSpdOperator<T>::InverseSpdOperator(const std::string& name,
                                          const MatrixX<T>& A)
    : LinearOperator<T>(name) {
  DRAKE_DEMAND(A.rows() == A.cols());
  size_ = A.rows();
  /* Factorize the matrix at construction so that calls to Multiply won't need
   to factorize again. */
  A_llt_ = A.llt();
}

template <typename T>
void InverseSpdOperator<T>::DoMultiply(
    const Eigen::Ref<const Eigen::SparseVector<T>>& x,
    Eigen::SparseVector<T>* y) const {
  VectorX<T> x_tmp(x);
  *y = A_llt_.solve(x_tmp).sparseView();
}

template <typename T>
void InverseSpdOperator<T>::DoMultiply(const Eigen::Ref<const VectorX<T>>& x,
                                       VectorX<T>* y) const {
  *y = A_llt_.solve(x);
}

}  // namespace internal
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::multibody::contact_solvers::internal::InverseSpdOperator)
