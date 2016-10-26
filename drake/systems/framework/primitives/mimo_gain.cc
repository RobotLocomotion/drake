#include "drake/systems/framework/primitives/mimo_gain.h"

#include "drake/common/eigen_autodiff_types.h"
#include "drake/common/eigen_types.h"

namespace drake {
namespace systems {

const int kNumStates = 0;

template <typename T>
MimoGain<T>::MimoGain(int size)
    : LinearSystemPlant<T>(MatrixX<T>::Zero(kNumStates, kNumStates),  // A
                           MatrixX<T>::Zero(kNumStates, size),        // B
                           MatrixX<T>::Zero(size, kNumStates),        // C
                           MatrixX<T>::Identity(size, size)) { }      // D

template <typename T>
MimoGain<T>::MimoGain(const Eigen::Ref<const MatrixX<T>> &D)
    : LinearSystemPlant<T>(MatrixX<T>::Zero(kNumStates, kNumStates),  // A
                           MatrixX<T>::Zero(kNumStates, D.cols()),    // B
                           MatrixX<T>::Zero(D.rows(), kNumStates),    // C
                           D) { }

template class DRAKE_EXPORT MimoGain<double>;
template class DRAKE_EXPORT MimoGain<AutoDiffXd>;

}  // namespace systems
}  // namespace drake
