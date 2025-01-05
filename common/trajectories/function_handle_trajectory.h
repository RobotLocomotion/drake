#pragma once

#include <functional>
#include <limits>
#include <memory>

#include "drake/common/default_scalars.h"
#include "drake/common/reset_after_move.h"
#include "drake/common/trajectories/trajectory.h"

namespace drake {
namespace trajectories {

/** FunctionHandleTrajectory takes a function, value = f(t), and provides a
Trajectory interface.

@tparam_default_scalar */
template <typename T>
class FunctionHandleTrajectory final : public Trajectory<T> {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(FunctionHandleTrajectory)

  /** Creates the FunctionHandleTrajectory.

  By default the created trajectory does not provide derivatives. If
  trajectory derivatives are required, call `set_derivative` to provide
  the function's derivatives.

  @param func The function to be used to evaluate the trajectory.
  @param rows The number of rows in the output of the function.
  @param cols The number of columns in the output of the function.
  @param start_time The start time of the trajectory.
  @param end_time The end time of the trajectory.
  @throws std::exception if func == nullptr, rows < 0, cols < 0, start_time >
  end_time, or if the function returns a matrix of the wrong size.
  */
  FunctionHandleTrajectory(
      std::function<MatrixX<T>(const T&)> func, int rows, int cols = 1,
      double start_time = -std::numeric_limits<double>::infinity(),
      double end_time = std::numeric_limits<double>::infinity());

  // TODO(russt): Consider offering an additional constructor that simply
  // evaluates `func` at `start_time` to determine the number of rows and
  // columns.

  ~FunctionHandleTrajectory() final;

  /** Sets a callback function that returns the derivative of the function.
  `func(t,order)` will only be called with `order > 0`. It is recommended that
  if the derivatives are not implemented for the requested order, the callback
  should throw an exception.

  The size of the output of @p func will be checked each time the derivative is
  evaluated, and a std::exception will be thrown if the size is incorrect.
  */
  void set_derivative(
      std::function<MatrixX<T>(const T& /*t*/, int /* order */)> func) {
    derivative_func_ = func;
  }

 private:
  // Trajectory overrides.
  std::unique_ptr<Trajectory<T>> DoClone() const final;
  MatrixX<T> do_value(const T& t) const final;
  bool do_has_derivative() const final { return derivative_func_ != nullptr; }
  // This method throws a std::exception if derivative_order != 0 and
  // derivative_func_ == nullptr.
  MatrixX<T> DoEvalDerivative(const T& t, int derivative_order) const final;
  // This method throws a std::exception if derivative_order != 0 and
  // derivative_func_ == nullptr.
  std::unique_ptr<Trajectory<T>> DoMakeDerivative(
      int derivative_order) const final;
  Eigen::Index do_rows() const final { return rows_; };
  Eigen::Index do_cols() const final { return cols_; };
  T do_start_time() const final { return start_time_; };
  T do_end_time() const final { return end_time_; };

  std::function<MatrixX<T>(const T&)> func_{};
  std::function<MatrixX<T>(const T&, int)> derivative_func_{};
  reset_after_move<int> rows_;
  reset_after_move<int> cols_;
  double start_time_;
  double end_time_;
};

}  // namespace trajectories
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class drake::trajectories::FunctionHandleTrajectory)
