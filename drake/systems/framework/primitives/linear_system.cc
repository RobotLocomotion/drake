#include "drake/systems/framework/primitives/linear_system.h"

#include "drake/common/eigen_autodiff_types.h"
#include "drake/common/eigen_types.h"

namespace drake {
namespace systems {

template <typename T>
LinearSystem<T>::LinearSystem(const Eigen::Ref<const MatrixX<T>>& A,
                              const Eigen::Ref<const MatrixX<T>>& B,
                              const Eigen::Ref<const MatrixX<T>>& C,
                              const Eigen::Ref<const MatrixX<T>>& D)
    : AffineSystem<T>(A, B, VectorX<T>::Zero(A.rows(), 1), C, D,
                      VectorX<T>::Zero(C.rows(), 1)) {}
// TODO(naveenoid): Modify constructor to accommodate 0 dimension systems;
// i.e. in initializing xDot0 and y0 with a zero matrix.
template class DRAKE_EXPORT LinearSystem<double>;
template class DRAKE_EXPORT LinearSystem<AutoDiffXd>;

}  // namespace systems
}  // namespace drake
