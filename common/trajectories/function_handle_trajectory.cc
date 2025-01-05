#include "drake/common/trajectories/function_handle_trajectory.h"

#include <utility>

#include "drake/common/trajectories/derivative_trajectory.h"

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

  // Call value() once to confirm that function returns the correct size. We
  // must do the check in the value() method, because the func_ could produce
  // differently sized outputs at different times.
  this->value(start_time);
}

template <typename T>
FunctionHandleTrajectory<T>::~FunctionHandleTrajectory() = default;

template <typename T>
std::unique_ptr<Trajectory<T>> FunctionHandleTrajectory<T>::DoClone() const {
  using Self = FunctionHandleTrajectory<T>;
  auto clone =
      std::make_unique<Self>(func_, rows_, cols_, start_time_, end_time_);
  clone->set_derivative(derivative_func_);
  return clone;
}

template <typename T>
MatrixX<T> FunctionHandleTrajectory<T>::do_value(const T& t) const {
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

template <typename T>
MatrixX<T> FunctionHandleTrajectory<T>::DoEvalDerivative(
    const T& t, int derivative_order) const {
  if (derivative_order == 0) {
    return this->value(t);
  }
  DRAKE_THROW_UNLESS(derivative_func_ != nullptr);
  MatrixX<T> derivative = derivative_func_(t, derivative_order);
  if (derivative.rows() != rows_ || derivative.cols() != cols_) {
    throw std::runtime_error(fmt::format(
        "The FunctionHandleTrajectory derivative callback returned a matrix "
        "of size {}x{}, but the constructor specified that the "
        "output should be of size {}x{}.",
        derivative.rows(), derivative.cols(), rows_.value(), cols_.value()));
  }
  return derivative;
}

template <typename T>
std::unique_ptr<Trajectory<T>> FunctionHandleTrajectory<T>::DoMakeDerivative(
    int derivative_order) const {
  if (derivative_order == 0) {
    return this->Clone();
  }
  DRAKE_THROW_UNLESS(derivative_func_ != nullptr);
  return std::make_unique<DerivativeTrajectory<T>>(*this, derivative_order);
}

}  // namespace trajectories
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class drake::trajectories::FunctionHandleTrajectory)
