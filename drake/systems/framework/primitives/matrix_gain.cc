#include "drake/systems/framework/primitives/matrix_gain.h"

#include "drake/common/eigen_autodiff_types.h"
#include "drake/common/eigen_types.h"

namespace drake {
namespace systems {

namespace {
const int kNumStates{0};
}  // namespace

template <typename T>
MatrixGain<T>::MatrixGain(int size)
    : MatrixGain<T>(MatrixX<T>::Identity(size, size)) {}

template <typename T>
MatrixGain<T>::MatrixGain(const MatrixX<T>& D)
    : LinearSystem<T>(MatrixX<T>::Zero(kNumStates, kNumStates),  // A
                      MatrixX<T>::Zero(kNumStates, D.cols()),    // B
                      MatrixX<T>::Zero(D.rows(), kNumStates),    // C
                      D) {}

template class DRAKE_EXPORT MatrixGain<double>;
template class DRAKE_EXPORT MatrixGain<AutoDiffXd>;

}  // namespace systems
}  // namespace drake
