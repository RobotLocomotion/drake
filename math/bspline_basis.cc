#include "drake/math/bspline_basis.h"

#include <algorithm>
#include <set>

#include "drake/common/text_logging.h"

using drake::MatrixX;
using drake::trajectories::PiecewisePolynomial;

namespace drake {
namespace math {
namespace {

std::vector<double> ConstructDefaultKnots(int order, int num_control_points) {
  const int num_knots{num_control_points + order};
  std::vector<double> knots(num_knots, 0.0);
  const double knot_interval =
      1.0 / static_cast<double>(num_control_points - (order - 1));
  for (int i = order; i < num_knots; ++i) {
    knots[i] = std::min(1.0, knots[i - 1] + knot_interval);
  }
  return knots;
}

template <typename T>
PiecewisePolynomial<T> BSplineOmega(int index, int order,
                                    const std::vector<double>& knots,
                                    const std::vector<double>& breaks) {
  const auto zero = PiecewisePolynomial<T>::ZeroOrderHold(
      breaks,
      {breaks.size(), PiecewisePolynomial<T>::CoefficientMatrix::Zero(1, 1)});
  if (knots[index + order - 1] - knots[index] <
      PiecewisePolynomial<T>::kEpsilonTime) {
    return zero;
  } else {
    std::vector<typename PiecewisePolynomial<T>::CoefficientMatrix> values(
        breaks.size(),
        typename PiecewisePolynomial<T>::CoefficientMatrix(1, 1));
    const int kNumBreaks(breaks.size());
    for (int i = 0; i < kNumBreaks; ++i) {
      values[i](0, 0) = (breaks[i] - knots[index]) /
                        (knots[index + order - 1] - knots[index]);
    }
    return PiecewisePolynomial<T>::FirstOrderHold(breaks, values);
  }
}

// Constructs the @p index-th B-spline of order @p order for the knot sequence
// @p knots. The result has the following properties:
//    - It is a piecewise polynomial of degree @p order - 1
//    - It is zero outside of the interval [@p knots[@p index], @p knots[@p
//      index + @p order]]
template <typename T>
PiecewisePolynomial<T> BSpline(int index, int order,
                               const std::vector<double>& knots) {
  const int kNumKnots(knots.size());
  DRAKE_THROW_UNLESS(index < kNumKnots);
  std::vector<double> breaks(kNumKnots, 0);
  std::vector<typename PiecewisePolynomial<T>::CoefficientMatrix> values(
      kNumKnots, PiecewisePolynomial<T>::CoefficientMatrix::Zero(1, 1));
  int breaks_index = 0;
  for (int i = 0; i < kNumKnots - 1; ++i) {
    // Add knots[i] to breaks if it is not equal to knots[i+1].
    if (knots[i + 1] > knots[i] + PiecewisePolynomial<T>::kEpsilonTime) {
      breaks[breaks_index] = knots[i];
      if (i == index) {
        values[breaks_index](0) = 1.0;
      }
      ++breaks_index;
    }
  }
  breaks[breaks_index] = knots.back();
  breaks.resize(breaks_index + 1);
  values.resize(breaks_index + 1);
  if (order == 1) {
    return PiecewisePolynomial<T>::ZeroOrderHold(breaks, values);
  } else {
    const auto one = PiecewisePolynomial<T>::ZeroOrderHold(
        breaks,
        {breaks.size(), PiecewisePolynomial<T>::CoefficientMatrix::Ones(1, 1)});
    return BSplineOmega<T>(index, order, knots, breaks) *
               BSpline<T>(index, order - 1, knots) +
           (one - BSplineOmega<T>(index + 1, order, knots, breaks)) *
               BSpline<T>(index + 1, order - 1, knots);
  }
}

}  // namespace

BsplineBasis::BsplineBasis(int order, std::vector<double> knots)
    : order_(order), num_control_points_(knots.size() - order), knots_(knots) {
  DRAKE_THROW_UNLESS(std::is_sorted(knots.begin(), knots.end()));
  for (int i = 0; i < num_control_points_; ++i) {
    basis_.push_back(BSpline<double>(i, order_, knots_));
  }
}

BsplineBasis::BsplineBasis(int order, int num_control_points)
    : BsplineBasis(order, ConstructDefaultKnots(order, num_control_points)) {}

PiecewisePolynomial<double> BsplineBasis::ConstructBsplineCurve(
    const std::vector<MatrixX<double>>& control_points) const {
  DRAKE_THROW_UNLESS(static_cast<int>(control_points.size()) ==
                     num_control_points_);
  const int control_point_rows = control_points.front().rows();
  const int control_point_cols = control_points.front().cols();
  const int num_segments = basis_.front().get_number_of_segments();
  std::vector<MatrixX<Polynomial<double>>> polynomials(num_segments);
  for (int segment_index = 0; segment_index < num_segments; ++segment_index) {
    polynomials[segment_index] = MatrixX<Polynomial<double>>::Zero(
        control_point_rows, control_point_cols);
    // TODO(avalenzu): Only do this for the elements of the basis whose
    // support includes the segment_index-th interval.
    for (int control_point_index = 0; control_point_index < num_control_points_;
         ++control_point_index) {
      DRAKE_THROW_UNLESS(control_points[control_point_index].rows() ==
                         control_point_rows);
      DRAKE_THROW_UNLESS(control_points[control_point_index].cols() ==
                         control_point_cols);
      for (int i = 0; i < control_point_rows; ++i) {
        for (int j = 0; j < control_point_cols; ++j) {
          polynomials[segment_index](i, j) +=
              basis_[control_point_index].getPolynomial(segment_index, 0, 0) *
              control_points[control_point_index](i, j);
        }
      }
    }
  }
  return PiecewisePolynomial<double>(polynomials,
                                     basis_.front().get_segment_times());
}

std::vector<int> BsplineBasis::ComputeActiveControlPointIndices(
    std::array<double, 2> interval) const {
  DRAKE_ASSERT(knots_.front() <= interval.front() + kEpsilonTime_);
  DRAKE_ASSERT(interval.back() <= knots_.back() + kEpsilonTime_);
  // If t ∈ [tᵣ, tᵣ₊₁), the only control points that contribute to the value of
  // the curve at t are P[r - p], ..., P[r - 1], P[r], where p = order - 1 is
  // the degree of the basis functions. We want to know which control points are
  // active over the interval [tₛ,  tₑ]. If we find the corresponding rₛ and
  // rₑ, then the active control points for the interval are
  // P[rₛ - p], ..., P[rₑ]
  //
  // Reference:
  // http://web.mit.edu/hyperbook/Patrikalakis-Maekawa-Cho/node17.html
  // We use a set to store the indices here so that we don't need to worry about
  // adding an index twice.
  std::set<int> active_control_point_indices;
  for (int r = num_control_points_ - 1; r >= 0; --r) {
    if (knots_[r] <= interval.back() + kEpsilonTime_) {
      // Add rₑ to the set.
      active_control_point_indices.insert(r);
      break;
    }
  }
  for (int r = *active_control_point_indices.begin(); r > 0; --r) {
    // Add {rₑ - 1, rₑ - 2, ..., rₛ} to the set.
    active_control_point_indices.insert(r);
    if (interval.front() > knots_[r] - kEpsilonTime_) {
      break;
    }
  }
  // Add {rₛ - 1, rₛ - 2, ..., rₛ - p} to the set.
  const int r_s = *active_control_point_indices.begin();
  for (int j = 1; j < order(); ++j) {
    active_control_point_indices.insert(r_s - j);
  }
  return std::vector<int>(active_control_point_indices.begin(),
                          active_control_point_indices.end());
}

std::vector<int> BsplineBasis::ComputeActiveControlPointIndices(
    double time) const {
  return ComputeActiveControlPointIndices({{time, time}});
}

double BsplineBasis::EvaluateBasisFunction(int index,
                                           double evaluation_time) const {
  DRAKE_ASSERT(0 <= index && index < num_control_points_);
  return basis_[index].value(evaluation_time)(0);
}

bool BsplineBasis::operator==(const BsplineBasis& other) const {
  return this->order() == other.order() && this->knots() == other.knots();
}

}  // namespace math
}  // namespace drake
