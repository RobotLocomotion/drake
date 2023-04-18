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

  virtual ~BezierCurve() = default;

  /** Returns the order of the curve (1 for linear, 2 for quadratic, etc.). */
  int order() const { return order_; }

  /** Returns the value of the ith basis function of `order` (1 for linear, 2
   for quadratic, etc) evaluated at `time`. The default value for the optional
   argument `order` is the `order()` of `this`. */
  T BernsteinBasis(int i, const T& time,
                   std::optional<int> order = std::nullopt) const;

  /** Returns a reference to the control points which define the curve. */
  const MatrixX<T>& control_points() const { return control_points_; }

  // Required methods for trajectories::Trajectory interface.

  std::unique_ptr<trajectories::Trajectory<T>> Clone() const override;

  /** Evaluates the curve at the given time.
  @warning If t does not lie in the range [start_time(), end_time()], the
           trajectory will silently be evaluated at the closest valid value of
           time to `time`. For example, `value(-1)` will return `value(0)` for
           a trajectory defined over [0, 1]. */
  MatrixX<T> value(const T& time) const override;

  MatrixX<symbolic::Expression> GetBezierExpression(
      symbolic::Variable time = symbolic::Variable("t")) const;

  /** Evaluates the curve at the given time. If clamp_time is true and t does
   not lie in the range [start_time(), end_time()], the trajectory will silently
   be evaluated at the closest valid value of time to `time`. For example,
   `value(-1)` will return `value(0)` for a trajectory defined over [0, 1]. When
   the curve is of type Expression, calling this method with time =
   symbolic::Variable("t") and clamp_time = false enables the extraction of the
   underlying polynomial expression of this curve. */
  MatrixX<T> value(const T& time, bool clamp_time) const;

  Eigen::Index rows() const override { return control_points_.rows(); }

  Eigen::Index cols() const override { return 1; }

  T start_time() const override { return start_time_; }

  T end_time() const override { return end_time_; }

 private:
  MatrixX<T> CalcDerivativePoints(int derivative_order) const;

  bool do_has_derivative() const override { return true; }

  MatrixX<T> DoEvalDerivative(const T& t, int derivative_order) const override;

  std::unique_ptr<trajectories::Trajectory<T>> DoMakeDerivative(
      int derivative_order) const override;

  template <typename T>
  MatrixX<T> EvaluateT(const T& time) const;

  double start_time_{};
  double end_time_{};
  MatrixX<T> control_points_;
  int order_{};
};
}  // namespace trajectories
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class drake::trajectories::BezierCurve)
