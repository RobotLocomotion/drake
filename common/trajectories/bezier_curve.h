#pragma once

#include <memory>
#include <optional>

#include "drake/common/eigen_types.h"
#include "drake/common/trajectories/trajectory.h"

namespace drake {
namespace trajectories {
/** A Bézier curve is defined by a set of control points p₀ through pₙ, where n
 is called the order of the curve (n = 1 for linear, 2 for quadratic, 3 for
 cubic, etc.). The first and last control points are always the endpoints of
 the curve; however, the intermediate control points (if any) generally do not
 lie on the curve, but the curve is guaranteed to stay within the convex hull
 of the control points.

 See also BsplineTrajectory. A B-spline can be thought of as a composition of
 overlapping Bézier curves (where each evaluation only depends on a local
 subset of the control points). In contrast, evaluating a Bézier curve will use
 all of the control points.

 @tparam_default_scalar
 */
template <typename T>
class BezierCurve final : public trajectories::Trajectory<T> {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(BezierCurve);

  /** Default initializer. Constructs an empty Bézier curve over the interval
   t ∈ [0, 1].*/
  BezierCurve() : BezierCurve<T>(0, 1, MatrixX<T>()) {}

  /** Constructs a Bézier curve over the interval t ∈ [`start_time`,
   `end_time`] with control points defined in the columns of `control_points`.
   @pre end_time >= start_time.
   */
  BezierCurve(double start_time, double end_time,
              const Eigen::Ref<const MatrixX<T>>& control_points);

  // TODO(russt): Add support for MatrixX control points, but only if we have a
  // use case for it.

  ~BezierCurve() final;

  /** Returns the order of the curve (1 for linear, 2 for quadratic, etc.). */
  int order() const { return control_points_.cols() - 1; }

  /** Returns the value of the ith basis function of `order` (1 for linear, 2
   for quadratic, etc) evaluated at `time`. The default value for the optional
   argument `order` is the `order()` of `this`. */
  T BernsteinBasis(int i, const T& time,
                   std::optional<int> order = std::nullopt) const;

  /** Returns a const reference to the control points which define the curve. */
  const MatrixX<T>& control_points() const { return control_points_; }

  /** Supports writing optimizations using the control points as decision
   variables.  This method returns the matrix, `M`, defining the control points
   of the `order` derivative in the form:
   <pre>
   derivative.control_points() = this.control_points() * M
   </pre>
   For instance, since we have
   <pre>
   derivative.control_points().col(k) = this.control_points() * M.col(k),
   </pre>
   constraining the kth control point of the `n`th derivative to be in [ub,
   lb], could be done with:
   @code
   auto M = curve.AsLinearInControlPoints(n);
   for (int i=0; i<curve.rows(); ++i) {
    prog.AddLinearConstraint(M.col(i).transpose(),
                             Vector1d(lb(i)),
                             Vector1d(ub(i)),
                             curve.row(i).transpose());
   }
   @endcode
   Iterating over the rows of the control points is the natural sparsity pattern
   here (since `M` is the same for all rows).  For instance, we also have
   <pre>
   derivative.control_points().row(k).T = M.T * this.control_points().row(k).T,
   </pre> or
   <pre>
   vec(derivative.control_points().T) = blockMT * vec(this.control_points().T),
   blockMT = [ M.T,   0, .... 0 ]
             [   0, M.T, 0, ... ]
             [      ...         ]
             [  ...    , 0, M.T ].
   </pre>
   @pre derivative_order >= 0. */
  Eigen::SparseMatrix<double> AsLinearInControlPoints(
      int derivative_order = 1) const;

  /** Evaluates the curve at the given time.
  @warning If t does not lie in the range [start_time(), end_time()], the
           trajectory will silently be evaluated at the closest valid value of
           time to `time`. For example, `value(-1)` will return `value(0)` for
           a trajectory defined over [0, 1]. */
  MatrixX<T> value(const T& t) const {
    // We shadowed the base class to add documentation, not to change logic.
    return Trajectory<T>::value(t);
  }

  /** Extracts the expanded underlying polynomial expression of this curve in
   terms of variable `time`. */
  VectorX<symbolic::Expression> GetExpression(
      symbolic::Variable time = symbolic::Variable("t")) const;

  /** Increases the order of the curve by 1. A Bézier curve of order n can be
   converted into a Bézier curve of order n + 1 with the same shape. The
   control points of `this` are modified to obtain the equivalent curve. */
  void ElevateOrder();

 private:
  // Trajectory overrides.
  std::unique_ptr<trajectories::Trajectory<T>> DoClone() const final;
  MatrixX<T> do_value(const T& t) const final;
  bool do_has_derivative() const final { return true; }
  MatrixX<T> DoEvalDerivative(const T& t, int derivative_order) const final;
  std::unique_ptr<trajectories::Trajectory<T>> DoMakeDerivative(
      int derivative_order) const final;
  Eigen::Index do_rows() const final { return control_points_.rows(); }
  Eigen::Index do_cols() const final { return 1; }
  T do_start_time() const final { return start_time_; }
  T do_end_time() const final { return end_time_; }

  /* Calculates the control points of the derivative curve. */
  MatrixX<T> CalcDerivativePoints(int derivative_order) const;

  VectorX<T> EvaluateT(const T& time) const;

  double start_time_{};
  double end_time_{};
  MatrixX<T> control_points_;
};
}  // namespace trajectories
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class drake::trajectories::BezierCurve);
