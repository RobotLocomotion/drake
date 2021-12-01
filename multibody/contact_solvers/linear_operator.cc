#include "drake/multibody/contact_solvers/linear_operator.h"

#include "drake/common/default_scalars.h"

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace internal {

template <typename T>
void LinearOperator<T>::DoMultiplyByTranspose(
    const Eigen::Ref<const Eigen::SparseVector<T>>&,
    Eigen::SparseVector<T>*) const {
  ThrowIfNotImplemented(__func__);
}

template <typename T>
void LinearOperator<T>::DoMultiplyByTranspose(
    const Eigen::Ref<const VectorX<T>>&, VectorX<T>*) const {
  ThrowIfNotImplemented(__func__);
}

template <typename T>
void LinearOperator<T>::DoAssembleMatrix(Eigen::SparseMatrix<T>*) const {
  ThrowIfNotImplemented(__func__);
}

template <typename T>
void LinearOperator<T>::DoAssembleMatrix(BlockSparseMatrix<T>*) const {
  ThrowIfNotImplemented(__func__);
}

}  // namespace internal
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::multibody::contact_solvers::internal::LinearOperator)
