#include "drake/systems/framework/primitives/mimo_gain.h"

#include "drake/common/eigen_autodiff_types.h"
#include "drake/common/eigen_types.h"

namespace drake {
namespace systems {

template <typename T>
MimoGain<T>::MimoGain(const Eigen::Ref<const MatrixX<T>> &D)
    : LinearSystemPlant<T>(VectorX<T>::Zero(0, 0),        // A
                           VectorX<T>::Zero(0, 0),        // B
                           VectorX<T>::Zero(D.rows(), 1), // C
                           D) { }

template class DRAKE_EXPORT MimoGain<double>;
template class DRAKE_EXPORT MimoGain<AutoDiffXd>;

}  // namespace systems
}  // namespace drake
