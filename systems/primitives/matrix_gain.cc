#include "drake/systems/primitives/matrix_gain.h"

#include "drake/common/default_scalars.h"

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
          SystemTypeTag<MatrixGain>{},
          Eigen::MatrixXd::Zero(kNumStates, kNumStates),  // A
          Eigen::MatrixXd::Zero(kNumStates, D.cols()),    // B
          Eigen::MatrixXd::Zero(D.rows(), kNumStates),    // C
          D,
          0.0 /* time_period */) {}

template <typename T>
template <typename U>
MatrixGain<T>::MatrixGain(const MatrixGain<U>& other)
    : MatrixGain<T>(other.D()) {}

}  // namespace systems
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::systems::MatrixGain)
