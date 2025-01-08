#pragma once

#include <memory>

#include "drake/common/default_scalars.h"
#include "drake/common/trajectories/trajectory.h"

namespace drake {
namespace trajectories {
namespace internal {

/* A WrappedTrajectory delegates all calls to a nested Trajectory object
maintained as a shared_ptr.
@tparam_default_scalar */
template <typename T>
class WrappedTrajectory final : public Trajectory<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(WrappedTrajectory);

  /* Wraps the given trajectory, which must not be nullptr. */
  explicit WrappedTrajectory(std::shared_ptr<const Trajectory<T>> trajectory);

  ~WrappedTrajectory() final;

  /* Returns the underlying Trajectory. */
  const Trajectory<T>* unwrap() const;

 private:
  // Trajectory overrides.
  std::unique_ptr<Trajectory<T>> DoClone() const final;
  MatrixX<T> do_value(const T& t) const final;
  bool do_has_derivative() const final;
  MatrixX<T> DoEvalDerivative(const T& t, int derivative_order) const final;
  std::unique_ptr<Trajectory<T>> DoMakeDerivative(
      int derivative_order) const final;
  Eigen::Index do_rows() const final;
  Eigen::Index do_cols() const final;
  T do_start_time() const final;
  T do_end_time() const final;

  std::shared_ptr<const Trajectory<T>> trajectory_;
};

}  // namespace internal
}  // namespace trajectories
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class drake::trajectories::internal::WrappedTrajectory);
