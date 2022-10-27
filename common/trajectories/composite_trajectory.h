#pragma once

#include <memory>
#include <vector>

#include "drake/common/copyable_unique_ptr.h"
#include "drake/common/default_scalars.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/common/trajectories/trajectory.h"

namespace drake {
namespace trajectories {

/** A %CompositeTrajectory stacks the values from one or more underlying
Trajectory objects into a single %Trajectory.

For sequencing trajectories in time, see PiecewiseTrajectory.

@tparam_default_scalars */
template <typename T>
class CompositeTrajectory final : public Trajectory<T> {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(CompositeTrajectory)

  /** Creates an empty trajectory. */
  explicit CompositeTrajectory(bool rowwise = true);

  ~CompositeTrajectory() final;

  /** Stacks another sub-Trajectory onto this.
  The size() of the value() matrix will grow but the [start, end] range will
  remain unchanged.
  @throws std::exception if the matrix dimension is incompatible. */
  void Append(std::unique_ptr<Trajectory<T>> traj);

  // Trajectory overrides.
  std::unique_ptr<Trajectory<T>> Clone() const final;
  MatrixX<T> value(const T& t) const final;
  Eigen::Index rows() const final;
  Eigen::Index cols() const final;
  T start_time() const final;
  T end_time() const final;

 private:
  // Trajectory overrides.
  bool do_has_derivative() const final;
  MatrixX<T> DoEvalDerivative(const T& t, int derivative_order) const final;
  std::unique_ptr<Trajectory<T>> DoMakeDerivative(
      int derivative_order) const final;

  bool rowwise_{};
  std::vector<copyable_unique_ptr<Trajectory<T>>> children_;
};

}  // namespace trajectories
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class drake::trajectories::CompositeTrajectory)
