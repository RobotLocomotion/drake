#pragma once

#include <memory>

#include "drake/common/copyable_unique_ptr.h"
#include "drake/common/default_scalars.h"
#include "drake/common/reset_after_move.h"
#include "drake/common/trajectories/trajectory.h"

namespace drake {
namespace trajectories {

/** Trajectory objects provide derivatives by implementing `DoEvalDerivative`
_and_ `DoMakeDerivative`. `DoEvalDerivative` evaluates the derivative value at
a point in time. `DoMakeDerivative` returns a new Trajectory object which
represents the derivative.

In some cases, it is easy to implement `DoEvalDerivative`, but difficult or
inefficient to implement `DoMakeDerivative` natively. And it may be just as
efficient to use `DoEvalDerivative` even in repeated evaluations of the
derivative. The %DerivativeTrajectory class helps with this case -- given a
`nominal` Trajectory, it provides a Trajectory interface that calls
`nominal.EvalDerivative()` to implement `Trajectory::value()`.

@tparam_default_scalar */
template <typename T>
class DerivativeTrajectory final : public Trajectory<T> {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(DerivativeTrajectory);

  /** Creates a DerivativeTrajectory representing the `derivative_order`
  derivatives of `nominal`. This constructor makes a Clone() of `nominal` and
  does not hold on to the reference.

  @throws std::exception if `!nominal.has_derivative()`.
  @throws std::exception if derivative_order < 0. */
  explicit DerivativeTrajectory(const Trajectory<T>& nominal,
                                int derivative_order = 1);

  ~DerivativeTrajectory() final;

 private:
  // Trajectory overrides.
  std::unique_ptr<Trajectory<T>> DoClone() const final;
  MatrixX<T> do_value(const T& t) const final;
  bool do_has_derivative() const final { return true; }
  MatrixX<T> DoEvalDerivative(const T& t, int derivative_order) const final;
  std::unique_ptr<Trajectory<T>> DoMakeDerivative(
      int derivative_order) const final;
  Eigen::Index do_rows() const final;
  Eigen::Index do_cols() const final;
  T do_start_time() const final;
  T do_end_time() const final;

  copyable_unique_ptr<Trajectory<T>> nominal_;
  reset_after_move<int> derivative_order_;
  reset_after_move<int> rows_;
  reset_after_move<int> cols_;
};

}  // namespace trajectories
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class drake::trajectories::DerivativeTrajectory);
