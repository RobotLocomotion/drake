#include "drake/math/bspline_curve.h"

#include <algorithm>

#include "drake/common/symbolic.h"
#include "drake/common/text_logging.h"
#include "drake/common/unused.h"

using drake::symbolic::Expression;
using drake::symbolic::Variable;

namespace drake {
namespace math {
namespace {
template <typename T, typename T_input>
std::vector<MatrixX<T>> CastControlPoints(
    const std::vector<MatrixX<T_input>>& control_points) {
  std::vector<MatrixX<T>> control_points_cast;
  std::transform(control_points.begin(), control_points.end(),
                 std::back_inserter(control_points_cast),
                 [](const MatrixX<T_input>& var) -> MatrixX<T> {
                   return var.template cast<T>();
                 });
  return control_points_cast;
}

MatrixX<Expression> ConstructExpressionForCurveValue(
    const BsplineBasis basis,
    const std::vector<MatrixX<Expression>>& control_points, double time) {
  // Compute the basis values at evaluation_time
  VectorX<double> basis_function_values(basis.order());
  const std::vector<int> active_control_point_indices =
      basis.ComputeActiveControlPointIndices(time);
  MatrixX<Expression> ret{control_points.front().rows(),
                          control_points.front().cols()};
  for (int i = 0; i < basis.order(); ++i) {
    ret += basis.EvaluateBasisFunction(active_control_point_indices[i], time) *
           control_points[active_control_point_indices[i]];
  }
  return ret;
}
}  // namespace

template <>
void BsplineCurve<double>::UpdatePiecewisePolynomial() {
  piecewise_polynomial_ = basis_.ConstructBsplineCurve(control_points_);
}

template <typename T>
void BsplineCurve<T>::UpdatePiecewisePolynomial() {
  // Piecewise polynomial doesn't support symbolic::{Expression,Variable}, so do
  // nothing here.
  // TODO(avalenzu): Update this so that it works for other numeric types.
}

template <typename T>
BsplineCurve<T>::BsplineCurve(const BsplineBasis& basis,
                              const std::vector<MatrixX<T>>& control_points)
    : basis_(basis), control_points_(control_points) {
  UpdatePiecewisePolynomial();
}

template <typename T>
template <typename T_input>
BsplineCurve<T>::BsplineCurve(
    const BsplineBasis& basis,
    const std::vector<MatrixX<T_input>>& control_points)
    : BsplineCurve(basis, CastControlPoints<T>(control_points)) {}

template <>
MatrixX<double> BsplineCurve<double>::value(double time) const {
  return piecewise_polynomial_->value(time);
}

template <>
MatrixX<Expression> BsplineCurve<Expression>::value(double time) const {
  return ConstructExpressionForCurveValue(basis_, control_points(), time);
}

template <typename T>
MatrixX<T> BsplineCurve<T>::InitialValue() const {
  return value(start_time());
}

template <>
void BsplineCurve<double>::InsertKnot(
    const std::vector<double>& additional_knots) {
  auto knots = this->knots();
  for (const auto& time : additional_knots) {
    DRAKE_THROW_UNLESS(knots.front() <= time && time <= knots.back());
    int i = 0;
    while (knots[i + degree()] < time) {
      ++i;
    }
    int k = i + degree() - 1;
    auto control_point_i_minus_1 = control_points()[i - 1];
    while (i <= k) {
      double a = (time - knots[i]) / (knots[i + degree()] - knots[i]);
      auto new_control_point{(1 - a) * control_point_i_minus_1 +
                             a * control_points()[i]};
      control_point_i_minus_1 = control_points()[i];
      control_points_[i] = new_control_point;
      ++i;
    }
    knots.insert(std::next(knots.begin(), i), time);
    control_points_.insert(std::next(control_points_.begin(), i),
                           control_point_i_minus_1);
  }
  basis_ = BsplineBasis(order(), knots);
  UpdatePiecewisePolynomial();
}

template <typename T>
void BsplineCurve<T>::InsertKnot(const std::vector<double>& time) {
  // TODO(avalenzu): Figure out the right way to handle this. This method is
  // only for BsplineCurve<double>.
  unused(time);
  DRAKE_THROW_UNLESS(false);
}

template <typename T>
BsplineCurve<T> BsplineCurve<T>::Derivative() const {
  std::vector<MatrixX<T>> derivative_control_points;
  std::vector<double> derivative_knots;
  const int num_derivative_knots = knots().size() - 1;
  for (int i = 1; i < num_derivative_knots; ++i) {
    derivative_knots.push_back(knots()[i]);
  }
  for (int i = 0; i < num_control_points() - 1; ++i) {
    derivative_control_points.push_back(
        degree() / (knots()[i + order()] - knots()[i + 1]) *
        (control_points()[i + 1] - control_points()[i]));
  }
  return BsplineCurve(BsplineBasis(order() - 1, derivative_knots),
                      derivative_control_points);
}

template <typename T>
bool BsplineCurve<T>::operator==(const BsplineCurve<T>& other) const {
  return this->basis() == other.basis() &&
         this->control_points() == other.control_points();
}

template <typename T>
math::BsplineCurve<T> BsplineCurve<T>::CopyBlock(int start_row, int start_col,
                                                 int block_rows,
                                                 int block_cols) const {
  std::vector<drake::MatrixX<T>> new_control_points{};
  new_control_points.reserve(num_control_points());
  std::transform(
      control_points().begin(), control_points().end(),
      std::back_inserter(new_control_points),
      [&](const drake::MatrixX<T>& control_point) -> drake::MatrixX<T> {
        return control_point.block(start_row, start_col, block_rows,
                                   block_cols);
      });
  return {basis(), new_control_points};
}

/// Creates a math::BsplineCurve consisting of the first
/// `n` elements of `original`.
/// @returns the newly created math::BsplineCurve.
/// @pre original.cols() == 1
template <typename T>
math::BsplineCurve<T> BsplineCurve<T>::CopyHead(int n) const {
  DRAKE_ASSERT(cols() == 1);
  return CopyBlock(0, 0, n, 1);
}

template class BsplineCurve<double>;
template class BsplineCurve<Expression>;
template BsplineCurve<Expression>::BsplineCurve(
    const BsplineBasis& basis, const std::vector<MatrixX<Variable>>&);
}  // namespace math
}  // namespace drake
