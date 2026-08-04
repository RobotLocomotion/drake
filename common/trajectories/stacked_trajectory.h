#pragma once

#include <memory>
#include <ranges>
#include <vector>

#include "drake/common/copyable_unique_ptr.h"
#include "drake/common/default_scalars.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/common/ranges.h"
#include "drake/common/reset_after_move.h"
#include "drake/common/trajectories/trajectory.h"

namespace drake {
namespace trajectories {

/** A %StackedTrajectory stacks the values from one or more underlying
Trajectory objects into a single %Trajectory, without changing the
`%start_time()` or `%end_time()`.

For sequencing trajectories in time instead, see CompositeTrajectory.

All of the underlying %Trajectory objects must have the same `%start_time()`
and `%end_time()`.

When constructed with `rowwise` set to true, all of the underlying %Trajectory
objects must have the same number of `%cols()` and the `value()` matrix will be
the **vstack** of the the trajectories in the order they were added.

When constructed with `rowwise` set to false, all of the underlying %Trajectory
objects must have the same number of `%rows()` and the `value()` matrix will be
the **hstack** of the the trajectories in the order they were added.

@tparam_default_scalar */
template <typename T>
class StackedTrajectory final : public Trajectory<T> {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(StackedTrajectory);

  /** Creates an empty trajectory.
  @param rowwise governs the stacking order */
  explicit StackedTrajectory(bool rowwise = true);

  ~StackedTrajectory() final;

  /** Stacks another sub-Trajectory onto this.
  Refer to the class overview documentation for details.
  @throws std::exception if the matrix dimension is incompatible. */
  void Append(const Trajectory<T>& traj);

  /** Returns true iff the stacking order is rowwise. */
  bool rowwise() const { return rowwise_; }

  /** Returns a view of the current children. Calling any non-const method on
  this trajectory will invalidate the view. */
  drake::range_view_of<const Trajectory<T>*> auto children() const {
    return children_ |
           std::views::transform(&copyable_unique_ptr<Trajectory<T>>::get);
  }

 private:
  void CheckInvariants() const;

  // We could make this public, if we ever need it for performance.
  void Append(std::unique_ptr<Trajectory<T>> traj);

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

  bool rowwise_{};
  std::vector<copyable_unique_ptr<Trajectory<T>>> children_;
  reset_after_move<int> rows_;
  reset_after_move<int> cols_;
};

}  // namespace trajectories
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class drake::trajectories::StackedTrajectory);
