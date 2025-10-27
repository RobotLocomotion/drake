#pragma once

#include <memory>
#include <vector>

#include "drake/common/copyable_unique_ptr.h"
#include "drake/common/eigen_types.h"
#include "drake/common/trajectories/piecewise_trajectory.h"

namespace drake {
namespace trajectories {
/** A "composite trajectory" is a series of trajectories joined end to end
 where the end time of one trajectory coincides with the starting time of the
 next.

 See also PiecewisePolynomial::ConcatenateInTime(), which might be preferred if
 all of the segments are PiecewisePolynomial.

 @tparam_default_scalar
 */
template <typename T>
class CompositeTrajectory final : public trajectories::PiecewiseTrajectory<T> {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(CompositeTrajectory);

  /** Constructs a composite trajectory from a list of Trajectories.
  @pre ∀i, `segments[i].get() != nullptr`.
  @pre ∀i, `segments[i+1].start_time() == segments[i].end_time()`.
  @pre ∀i, `segments[i].rows() == segments[0].rows()` and `segments[i].cols() ==
  segments[0].cols()`. */
  explicit CompositeTrajectory(
      std::vector<copyable_unique_ptr<Trajectory<T>>> segments);

  ~CompositeTrajectory() final;

  /** Evaluates the curve at the given time.
  @warning If t does not lie in the range [start_time(), end_time()], the
  trajectory will silently be evaluated at the closest valid value of time to
  `time`. For example, `value(-1)` will return `value(0)` for a trajectory
  defined over [0, 1]. */
  MatrixX<T> value(const T& t) const {
    // We shadowed the base class to add documentation, not to change logic.
    return PiecewiseTrajectory<T>::value(t);
  }

  /** Returns a reference to the `segment_index` trajectory. */
  const Trajectory<T>& segment(int segment_index) const {
    DRAKE_DEMAND(segment_index >= 0);
    DRAKE_DEMAND(segment_index < this->get_number_of_segments());
    return *segments_[segment_index];
  }

  /** Constructs a composite trajectory from a list of trajectories whose start
  and end times may not coincide, by translating their start and end times.
  @pre ∀i, `segments[i].get() != nullptr`.
  @pre ∀i, `segments[i].rows() == segments[0].rows()` and `segments[i].cols() ==
  segments[0].cols()`. */
  static CompositeTrajectory<T> AlignAndConcatenate(
      const std::vector<copyable_unique_ptr<Trajectory<T>>>& segments);

 private:
  // Trajectory overrides.
  std::unique_ptr<trajectories::Trajectory<T>> DoClone() const final;
  MatrixX<T> do_value(const T& t) const final;
  bool do_has_derivative() const final;
  MatrixX<T> DoEvalDerivative(const T& t, int derivative_order) const final;
  std::unique_ptr<trajectories::Trajectory<T>> DoMakeDerivative(
      int derivative_order) const final;
  Eigen::Index do_rows() const final;
  Eigen::Index do_cols() const final;

  std::vector<copyable_unique_ptr<Trajectory<T>>> segments_{};
};

}  // namespace trajectories
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class drake::trajectories::CompositeTrajectory);
