#pragma once

#include <limits>
#include <memory>
#include <vector>

#include <Eigen/Core>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/common/trajectories/trajectory.h"

namespace drake {
namespace trajectories {

/** A DiscreteTimeTrajectory is a Trajectory whose value is only defined at
discrete time points.  Calling `value()` at a time that is not equal to one of
those times (up to a tolerance) will throw.  This trajectory does *not* have
well-defined time-derivatives.

In some applications, it may be preferable to use
PiecewisePolynomial<T>::ZeroOrderHold instead of a DiscreteTimeTrajectory (and
we offer a method here to easily convert).  Note if the breaks are periodic,
then one can also achieve a similar result in a Diagram by using the
DiscreteTimeTrajectory in a TrajectorySource and connecting a ZeroOrderHold
system to the output port, but remember that this will add discrete state to
your diagram.

So why not always use the zero-order hold (ZOH) trajectory?  This class forces
us to be more precise in our implementations.  For instance, consider the case
of a solution to a discrete-time finite-horizon linear quadratic regulator
(LQR) problem. In this case, the solution to the Riccati equation is a
DiscreteTimeTrajectory, K(t).  Implementing
@verbatim
x(t) -> MatrixGain(-K(t)) -> u(t)
@endverbatim
in a block diagram is perfectly correct, and if the u(t) is only connected to
the original system that it was designed for, then K(t) will only get evaluated
at the defined sample times, and all is well.  But if you wire it up to a
continuous-time system, then K(t) may be evaluated at arbitrary times, and may
throw.  If one wishes to use the K(t) solution on a continuous-time system, then
we can use
@verbatim
x(t) -> MatrixGain(-K(t)) -> ZOH -> u(t).
@endverbatim
This is different, and *more correct* than implementing K(t) as a zero-order
hold trajectory, because in this version, both K(t) and the inputs x(t) will
only be evaluated at the discrete-time input.  If `t_s` was the most recent
discrete sample time, then this means u(t) = -K(t_s)*x(t_s) instead of u(t) =
-K(t_s)*x(t).  Using x(t_s) and having a true zero-order hold on u(t) is the
correct model for the discrete-time LQR result.

@tparam_default_scalar
*/
template <typename T>
class DiscreteTimeTrajectory final : public Trajectory<T> {
 public:
  // We are final, so this is okay.
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(DiscreteTimeTrajectory);

  /** Default constructor creates the empty trajectory. */
  DiscreteTimeTrajectory() = default;

  /** Constructs a trajectory of vector @p values at the specified @p times.
  @pre @p times must differ by more than @p time_comparison_tolerance and be
  monotonically increasing.
  @pre @p values must have times.size() columns.
  @pre @p time_comparison_tolerance must be >= 0.
  @throw if T=symbolic:Expression and @p times are not constants.
  @pydrake_mkdoc_identifier{Eigen} */
  DiscreteTimeTrajectory(const Eigen::Ref<const VectorX<T>>& times,
                         const Eigen::Ref<const MatrixX<T>>& values,
                         double time_comparison_tolerance =
                             std::numeric_limits<double>::epsilon());

  /** Constructs a trajectory of matrix @p values at the specified @p times.
  @pre @p times should differ by more than @p time_comparison_tolerance and be
  monotonically increasing.
  @pre @p values must have times.size() elements, each with the same number of
       rows and columns.
  @pre @p time_comparison_tolerance must be >= 0.
  @throw if T=symbolic:Expression and @p times are not constants.
  @pydrake_mkdoc_identifier{stdvector} */
  DiscreteTimeTrajectory(std::vector<T> times, std::vector<MatrixX<T>> values,
                         double time_comparison_tolerance =
                             std::numeric_limits<double>::epsilon());

  ~DiscreteTimeTrajectory() final;

  /** Converts the discrete-time trajectory using
  PiecewisePolynomial<T>::ZeroOrderHold(). */
  PiecewisePolynomial<T> ToZeroOrderHold() const;

  /** The trajectory is only defined at finite sample times.  This method
  returns the tolerance used determine which time sample (if any) matches a
  query time on calls to value(t). */
  double time_comparison_tolerance() const;

  /** Returns the number of discrete times where the trajectory value is
  defined. */
  int num_times() const;

  /** Returns the times where the trajectory value is defined. */
  const std::vector<T>& get_times() const;

  /** Returns the value of the trajectory at @p t.
  @throws std::exception if t is not within tolerance of one of the sample
  times. */
  MatrixX<T> value(const T& t) const {
    // We shadowed the base class to add documentation, not to change logic.
    return Trajectory<T>::value(t);
  }

  /** Returns the number of rows in the MatrixX<T> returned by value().
  @pre num_times() > 0. */
  Eigen::Index rows() const {
    // We shadowed the base class to add documentation, not to change logic.
    return Trajectory<T>::rows();
  }

  /** Returns the number of cols in the MatrixX<T> returned by value().
  @pre num_times() > 0. */
  Eigen::Index cols() const {
    // We shadowed the base class to add documentation, not to change logic.
    return Trajectory<T>::cols();
  }

  /** Returns the minimum value of get_times().
  @pre num_times() > 0. */
  T start_time() const {
    // We shadowed the base class to add documentation, not to change logic.
    return Trajectory<T>::start_time();
  }

  /** Returns the maximum value of get_times().
  @pre num_times() > 0. */
  T end_time() const {
    // We shadowed the base class to add documentation, not to change logic.
    return Trajectory<T>::end_time();
  }

 private:
  // Trajectory overrides.
  std::unique_ptr<Trajectory<T>> DoClone() const final;
  MatrixX<T> do_value(const T& t) const final;
  Eigen::Index do_rows() const final;
  Eigen::Index do_cols() const final;
  T do_start_time() const final;
  T do_end_time() const final;

  std::vector<T> times_;
  std::vector<MatrixX<T>> values_;
  double time_comparison_tolerance_{};
};

}  // namespace trajectories
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class drake::trajectories::DiscreteTimeTrajectory);
