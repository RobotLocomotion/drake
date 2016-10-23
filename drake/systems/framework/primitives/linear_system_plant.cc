#include "drake/systems/framework/primitives/linear_system_plant.h"

#include "drake/common/eigen_autodiff_types.h"
#include "drake/common/eigen_types.h"

namespace drake {
namespace systems {

template <typename T>
LinearSystemPlant<T>::LinearSystemPlant(
    const Eigen::Ref<const MatrixX<T>> &A,
    const Eigen::Ref<const MatrixX<T>> &B,
    const Eigen::Ref<const MatrixX<T>> &C,
    const Eigen::Ref<const MatrixX<T>> &D) :
    AffineSystemPlant<T>(A, B,
                         VectorX<T>::Zero(A.rows(), A.cols()),
                         C, D,
                         VectorX<T>::Zero(C.rows(), C.cols())) {
}
template class DRAKE_EXPORT LinearSystemPlant<double>;
template class DRAKE_EXPORT LinearSystemPlant<AutoDiffXd>;

}  // namespace systems
}  // namespace drake
