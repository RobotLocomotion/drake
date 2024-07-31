#pragma once

#include <memory>
#include <vector>

#include <Eigen/Core>

#include "drake/common/copyable_unique_ptr.h"
#include "drake/common/default_scalars.h"
#include "drake/common/trajectories/trajectory.h"

namespace drake {
namespace trajectories {

/**
 * A trajectory defined by a path and timing trajectory.
 *
 * Using a path of form `r(s)` and a time_scaling of the form `s(t)`, a full
 * trajectory of form `q(t) = r(s(t))` is modeled.
 *
 * @tparam_default_scalar
 */
template <typename T>
class PathParameterizedTrajectory final : public Trajectory<T> {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(PathParameterizedTrajectory);

  /** Constructs a trajectory with the given `path` and `time_scaling`.
  @pre time_scaling.rows() == time_scaling.cols() == 1 */
  PathParameterizedTrajectory(const Trajectory<T>& path,
                              const Trajectory<T>& time_scaling);

  ~PathParameterizedTrajectory() final = default;

  // Required methods for trajectories::Trajectory interface.
  std::unique_ptr<trajectories::Trajectory<T>> Clone() const override;

  /** Evaluates the PathParameterizedTrajectory at the given time t.
  @param t The time at which to evaluate the %PathParameterizedTrajectory.
  @return The matrix of evaluated values.
  @pre If T == symbolic::Expression, `t.is_constant()` must be true.
  @warning If t does not lie in the range [start_time(), end_time()], the
           trajectory will silently be evaluated at the closest
           valid value of time to t. For example, `value(-1)` will return
           `value(0)` for a trajectory defined over [0, 1]. */
  MatrixX<T> value(const T& t) const override;

  Eigen::Index rows() const override { return path_->rows(); }

  Eigen::Index cols() const override { return path_->cols(); }

  T start_time() const override { return time_scaling_->start_time(); }

  T end_time() const override { return time_scaling_->end_time(); }

  /** Returns the path of this trajectory. */
  const trajectories::Trajectory<T>& path() const { return *path_; }

  /** Returns the time_scaling of this trajectory. */
  const trajectories::Trajectory<T>& time_scaling() const {
    return *time_scaling_;
  }

 private:
  // Evaluates the %PathParameterizedTrajectory derivative at the given time @p
  // t. Returns the nth derivative, where `n` is the value of @p
  // derivative_order.
  //
  // @warning This method comes with the same caveats as value(). See value()
  // @pre derivative_order must be non-negative.
  MatrixX<T> DoEvalDerivative(const T& t, int derivative_order) const override;

  // Uses DerivativeTrajectory to provide a derivative object.
  std::unique_ptr<Trajectory<T>> DoMakeDerivative(
      int derivative_order) const final;

  // Evaluates the Bell Polynomial B_n,k(x) for use in calculating the
  // derivative.
  // @pre n and k must be non-negative and the length of x must be at least n.
  T BellPolynomial(int n, int k, const VectorX<T>& x) const;

  bool do_has_derivative() const override {
    return path_->has_derivative() && time_scaling_->has_derivative();
  }

  copyable_unique_ptr<Trajectory<T>> path_;
  copyable_unique_ptr<Trajectory<T>> time_scaling_;
};

}  // namespace trajectories
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class drake::trajectories::PathParameterizedTrajectory);
