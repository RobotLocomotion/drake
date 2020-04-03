#pragma once

#include <memory>
#include <vector>

#include "drake/common/drake_bool.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/common/trajectories/trajectory.h"
#include "drake/math/bspline_basis.h"

namespace drake {
namespace trajectories {
/** Represents a B-spline curve using a given `basis` with ordered
`control_points` such that each control point is a matrix in ℝʳᵒʷˢ ˣ ᶜᵒˡˢ.
@see math::BsplineBasis */
template <typename T>
class BsplineTrajectory final : public trajectories::Trajectory<T> {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(BsplineTrajectory);

  /** Constructs a B-spline trajectory with the given `basis` and
  `control_points`.
  @pre control_points.size() == basis.num_basis_functions() */
  BsplineTrajectory(math::BsplineBasis<double> basis,
                    std::vector<MatrixX<T>> control_points);

  virtual ~BsplineTrajectory() = default;

  // Required methods for trajectories::Trajectory interface.
  std::unique_ptr<trajectories::Trajectory<T>> Clone() const override;

  /** Evaluates the BsplineTrajectory at the given time t.
  @param t The time at which to evaluate the %PiecewisePolynomial.
  @return The matrix of evaluated values.
  @warning If t does not lie in the range [start_time(), end_time()], the
           trajectory will silently be evaluated at the closest
           valid value of time to t. For example, `value(-1)` will return
           `value(0)` for a trajectory defined over [0, 1]. */
  MatrixX<T> value(const T& time) const override;

  // TODO(avalenzu): The default parameter here violates the GSG, but is
  // included to match the parent class. It should be removed along with the
  // corresponding one in trajectories::Trajectory.
  std::unique_ptr<trajectories::Trajectory<T>> MakeDerivative(
      int derivative_order = 1) const override;

  Eigen::Index rows() const override { return control_points()[0].rows(); }

  Eigen::Index cols() const override { return control_points()[0].cols(); }

  T start_time() const override { return basis_.initial_parameter_value(); }

  T end_time() const override { return basis_.final_parameter_value(); }

  // Other methods
  /** Returns the number of control points in this curve. */
  int num_control_points() const { return basis_.num_basis_functions(); }

  /** Returns the control points of this curve. */
  const std::vector<MatrixX<T>>& control_points() const {
    return control_points_;
  }

  /** Returns this->value(this->start_time()) */
  MatrixX<T> InitialValue() const;

  /** Returns this->value(this->end_time()) */
  MatrixX<T> FinalValue() const;

  /** Returns the basis of this curve. */
  const math::BsplineBasis<double>& basis() const { return basis_; }

  /** Adds new knots at the specified `additional_knots` without changing the
  behavior of the trajectory. The basis and control points of the trajectory are
  adjusted such that it produces the same value for any valid time before and
  after this method is called. The resulting trajectory is guaranteed to have
  the same level of continuity as the original, even if knot values are
  duplicated. Note that `additional_knots` need not be sorted.
  @pre start_time() <= t <= end_time() for all t in `additional_knots` */
  void InsertKnots(const std::vector<double>& additional_knots);

  /** Returns a new BsplineTrajectory that uses the same basis as `this`, and
  whose control points are the result of calling `select(point)` on each `point`
  in `this->control_points()`.*/
  BsplineTrajectory<T> CopyWithSelector(
      const std::function<MatrixX<T>(const MatrixX<T>&)>& select) const;

  /** Returns a new BsplineTrajectory that uses the same basis as `this`, and
  whose control points are the result of calling
  `point.block(start_row, start_col, block_rows, block_cols)` on each `point`
  in `this->control_points()`.*/
  BsplineTrajectory<T> CopyBlock(int start_row, int start_col,
                                       int block_rows, int block_cols) const;

  /** Returns a new BsplineTrajectory that uses the same basis as `this`, and
  whose control points are the result of calling `point.head(n)` on each `point`
  in `this->control_points()`.
  @pre this->cols() == 1
  @pre control_points()[0].head(n) must be a valid operation. */
  BsplineTrajectory<T> CopyHead(int n) const;

  boolean<T> operator==(const BsplineTrajectory<T>& other) const;

 private:
  math::BsplineBasis<double> basis_;
  std::vector<MatrixX<T>> control_points_;
};
}  // namespace trajectories
}  // namespace drake
