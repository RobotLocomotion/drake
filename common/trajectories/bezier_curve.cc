#include "drake/common/trajectories/bezier_curve.h"

#include "drake/math/binomial_coefficient.h"

namespace drake {
namespace trajectories {

template <typename T>
BezierCurve<T>::BezierCurve(double start_time, double end_time,
                            const Eigen::Ref<const MatrixX<T>>& control_points)
    : start_time_{start_time},
      end_time_{end_time},
      control_points_{control_points},
      order_{static_cast<int>(control_points.cols()) - 1} {
  DRAKE_DEMAND(end_time >= start_time);
}

template <typename T>
T BezierCurve<T>::BernsteinBasis(int i, const T& time,
                                 std::optional<int> order) const {
  using std::pow;
  int n = order.value_or(order_);
  int coeff = math::BinomialCoefficient(n, i);
  T s = (time - start_time_) / (end_time_ - start_time_);
  return coeff * pow(s, i) * pow(1 - s, n - i);
}

template <typename T>
std::unique_ptr<Trajectory<T>> BezierCurve<T>::Clone() const {
  return std::make_unique<BezierCurve<T>>(start_time_, end_time_,
                                          control_points_);
}

template <typename T>
MatrixX<T> BezierCurve<T>::value(const T& time) const {
  using std::clamp;
  return EvaluateT(clamp(time, T{start_time_}, T{end_time_}));
}

template <typename T>
MatrixX<symbolic::Expression>
BezierCurve<symbolic::Expression>::GetBezierExpression(
    symbolic::Variable time) const {
  return EvaluateT(symbolic::Expression(time));
}

template <typename T>
MatrixX<symbolic::Expression> BezierCurve<T>::GetBezierExpression(
    symbolic::Variable time) const {
  MatrixX<symbolic::Expression> control_points{control_points_.rows(),
                                               control_points_.cols()};
  for (int r = 0; r < control_points.rows(); ++r) {
    for (int c = 0; c < control_points.cols(); ++c) {
      switch (this) {
        case (std::is_same<T, double>::value()):
          control_points(r, c) = symbolic::Expression(control_points_(r, c));
          break;
        case (std::is_same < T, AutoDiffXd>::value()):
          control_points(r, c) = symbolic::Expression(control_points_(r, c).value());
      }
    }
  }
  return EvaluateT(symbolic::Expression(time));
}

template <typename T>
MatrixX<T> BezierCurve<T>::CalcDerivativePoints(int derivative_order) const {
  DRAKE_DEMAND(derivative_order <= order_);
  int n = order_;
  MatrixX<T> points =
      (control_points_.rightCols(order_) - control_points_.leftCols(order_)) *
      order_ / (end_time_ - start_time_);
  for (int i = 1; i < derivative_order; ++i) {
    n -= 1;
    points = (points.rightCols(n) - points.leftCols(n)).eval() * n /
             (end_time_ - start_time_);
  }
  return points;
}

template <typename T>
MatrixX<T> BezierCurve<T>::DoEvalDerivative(const T& time,
                                            int derivative_order) const {
  DRAKE_DEMAND(derivative_order >= 0);
  if (derivative_order == 0) {
    return this->value(time);
  }
  if (derivative_order > order_) {
    return VectorX<T>::Zero(rows());
  }

  MatrixX<T> points = CalcDerivativePoints(derivative_order);
  using std::clamp;
  const T ctime = clamp(time, T{start_time_}, T{end_time_});

  MatrixX<T> v = VectorX<T>::Zero(rows());
  for (int i = 0; i < points.cols(); ++i) {
    v += BernsteinBasis(i, ctime, order_ - derivative_order) * points.col(i);
  }
  return v;
}

template <typename T>
MatrixX<T> BezierCurve<T>::EvaluateT(const T& time) const {
  MatrixX<T> v = VectorX<T>::Zero(rows());
  for (int i = 0; i < control_points_.cols(); ++i) {
    v += BernsteinBasis(i, time) * control_points_.col(i);
  }
  return v;
}

template <typename T>
std::unique_ptr<Trajectory<T>> BezierCurve<T>::DoMakeDerivative(
    int derivative_order) const {
  DRAKE_DEMAND(derivative_order >= 0);
  if (derivative_order == 0) {
    return this->Clone();
  }
  if (derivative_order > order_) {
    // Then return the zero curve.
    return std::make_unique<BezierCurve<T>>(start_time_, end_time_,
                                            VectorX<T>::Zero(rows()));
  }

  return std::make_unique<BezierCurve<T>>(
      start_time_, end_time_, CalcDerivativePoints(derivative_order));
}

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class BezierCurve);

}  // namespace trajectories
}  // namespace drake
