#include "drake/systems/primitives/matrix_gain.h"

#include "drake/common/autodiff_overloads.h"
#include "drake/common/eigen_autodiff_types.h"
#include "drake/common/eigen_types.h"
#include "drake/common/symbolic.h"

namespace drake {
namespace systems {

namespace {
constexpr int kNumStates{0};
}  // namespace

template <typename T>
MatrixGain<T>::MatrixGain(int size)
    : MatrixGain<T>(Eigen::MatrixXd::Identity(size, size)) {}

template <typename T>
MatrixGain<T>::MatrixGain(const Eigen::MatrixXd& D)
    : LinearSystem<T>(
          SystemTypeTag<systems::MatrixGain>{},
          Eigen::MatrixXd::Zero(kNumStates, kNumStates),  // A
          Eigen::MatrixXd::Zero(kNumStates, D.cols()),    // B
          Eigen::MatrixXd::Zero(D.rows(), kNumStates),    // C
          D,
          0.0 /* time_period */) {}

template <typename T>
template <typename U>
MatrixGain<T>::MatrixGain(const MatrixGain<U>& other)
    : MatrixGain<T>(other.D()) {}

template class MatrixGain<double>;
template class MatrixGain<AutoDiffXd>;
template class MatrixGain<symbolic::Expression>;

}  // namespace systems
}  // namespace drake
