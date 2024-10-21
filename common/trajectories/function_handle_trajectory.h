#pragma once

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

  @param func The function to be used to evaluate the trajectory.
  @param rows The number of rows in the output of the function.
  @param cols The number of columns in the output of the function.
  @param start_time The start time of the trajectory.
  @param end_time The end time of the trajectory.
  @throws std::exception if func == nullptr, rows < 0, cols < 0, or start_time >
  end_time.
  */
  FunctionHandleTrajectory(
      const std::function<MatrixX<T>(const T&)>& func, int rows, int cols = 1,
      double start_time = -std::numeric_limits<double>::infinity(),
      double end_time = std::numeric_limits<double>::infinity());

  // TODO(russt): Consider offering an additional constructor that simply
  // evaluates `func` at `start_time` to determine the number of rows and
  // columns.

  ~FunctionHandleTrajectory() final;

  // Trajectory overrides.
  std::unique_ptr<Trajectory<T>> Clone() const final;
  MatrixX<T> value(const T& t) const final;
  Eigen::Index rows() const final { return rows_; };
  Eigen::Index cols() const final { return cols_; };
  T start_time() const final { return start_time_; };
  T end_time() const final { return end_time_; };

 private:
  // Trajectory overrides.
  bool do_has_derivative() const final { return false; }

  // TODO(russt): Support derivatives, potentially by having a setter method
  // which can pass an additional function handle that explicitly represents
  // the derivative (and using DerivativeTrajectory to implement
  // DoMakeDerivative).

  std::function<MatrixX<T>(const T&)> func_{};
  reset_after_move<int> rows_;
  reset_after_move<int> cols_;
  double start_time_;
  double end_time_;
};

}  // namespace trajectories
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class drake::trajectories::FunctionHandleTrajectory)
