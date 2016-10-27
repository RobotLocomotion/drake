#include "drake/systems/framework/primitives/mimo_gain.h"

#include "drake/common/eigen_autodiff_types.h"
#include "drake/common/eigen_types.h"

namespace drake {
namespace systems {

namespace {
const int kNumStates{0};
}  // namespace

template <typename T>
MimoGain<T>::MimoGain(int size)
    : MimoGain<T>(MatrixX<T>::Identity(size, size)) {}

template <typename T>
MimoGain<T>::MimoGain(const Eigen::Ref<const MatrixX<T>>& D)
    : LinearSystem<T>(MatrixX<T>::Zero(kNumStates, kNumStates),  // A
                      MatrixX<T>::Zero(kNumStates, D.cols()),    // B
                      MatrixX<T>::Zero(D.rows(), kNumStates),    // C
                      D) {}

template class DRAKE_EXPORT MimoGain<double>;
template class DRAKE_EXPORT MimoGain<AutoDiffXd>;

}  // namespace systems
}  // namespace drake
