#include "drake/multibody/solvers/linear_operator.h"

#include "drake/common/default_scalars.h"

namespace drake {
namespace multibody {
namespace solvers {

#define DRAKE_THROW_IF_NOT_IMPLEMENTED() ThrowIfNotImplemented(__func__)

template <typename T>
void LinearOperator<T>::DoMultiplyByTranspose(const VectorX<T>&,
                                              VectorX<T>*) const {
  DRAKE_THROW_IF_NOT_IMPLEMENTED();
}

template <typename T>
void LinearOperator<T>::DoMultiplyByTranspose(const Eigen::SparseVector<T>&,
                                              Eigen::SparseVector<T>*) const {
  DRAKE_THROW_IF_NOT_IMPLEMENTED();
}

template <typename T>
void LinearOperator<T>::AssembleMatrix(Eigen::SparseMatrix<T>*) const {
  DRAKE_THROW_IF_NOT_IMPLEMENTED();
}

}  // namespace solvers
}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::multibody::solvers::LinearOperator)
