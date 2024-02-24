#include "drake/common/trajectories/function_handle_trajectory.h"

#include <utility>

namespace drake {
namespace trajectories {

template <typename T>
FunctionHandleTrajectory<T>::FunctionHandleTrajectory(
    std::function<MatrixX<T>(const T&)> func, int rows, int cols,
    double start_time, double end_time)
    : func_(std::move(func)),
      rows_(rows),
      cols_(cols),
      start_time_(start_time),
      end_time_(end_time) {
  if (rows_ == 0 || cols_ == 0) {
    // Handle the empty (default) case; which can happen after a move operation.
    return;
  }

  DRAKE_THROW_UNLESS(func_ != nullptr);
  DRAKE_THROW_UNLESS(rows >= 0);
  DRAKE_THROW_UNLESS(cols >= 0);
  DRAKE_THROW_UNLESS(start_time <= end_time);

  // Call value() once to confirm that function returns the correct size.
  value(start_time);
}

template <typename T>
FunctionHandleTrajectory<T>::~FunctionHandleTrajectory() = default;

template <typename T>
std::unique_ptr<Trajectory<T>> FunctionHandleTrajectory<T>::Clone() const {
  using Self = FunctionHandleTrajectory<T>;
  return std::make_unique<Self>(func_, rows_, cols_, start_time_, end_time_);
}

template <typename T>
MatrixX<T> FunctionHandleTrajectory<T>::value(const T& t) const {
  if (rows_ == 0 || cols_ == 0) {
    // Handle the empty (default) case; which can happen after a move operation.
    return MatrixX<T>::Zero(0, 0);
  }
  MatrixX<T> result = func_(t);
  if (result.rows() != rows_ || result.cols() != cols_) {
    throw std::runtime_error(fmt::format(
        "The FunctionHandleTrajectory callback returned a matrix "
        "of size {}x{}, but the constructor specified that the "
        "output should be of size {}x{}.",
        result.rows(), result.cols(), rows_.value(), cols_.value()));
  }
  return result;
}

}  // namespace trajectories
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class drake::trajectories::FunctionHandleTrajectory)
