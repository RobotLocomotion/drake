#include "drake/common/trajectories/piecewise_polynomial.h"

#include <algorithm>
#include <memory>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_throw.h"

using std::runtime_error;
using std::vector;

namespace drake {
namespace trajectories {

template <typename T>
PiecewisePolynomial<T>::PiecewisePolynomial(
    std::vector<PolynomialMatrix> const& polynomials,
    std::vector<double> const& breaks)
    : PiecewiseTrajectory<T>(breaks), polynomials_(polynomials) {
  DRAKE_ASSERT(breaks.size() == (polynomials.size() + 1));
  for (int i = 1; i < this->get_number_of_segments(); i++) {
    if (polynomials[i].rows() != polynomials[0].rows())
      throw std::runtime_error(
          "The polynomial matrix for each segment must have the same number of "
          "rows.");
    if (polynomials[i].cols() != polynomials[0].cols())
      throw std::runtime_error(
          "The polynomial matrix for each segment must have the same number of "
          "columns.");
  }
}

template <typename T>
PiecewisePolynomial<T>::PiecewisePolynomial(
    std::vector<PolynomialType> const& polynomials,
    std::vector<double> const& breaks)
    : PiecewiseTrajectory<T>(breaks) {
  DRAKE_ASSERT(breaks.size() == (polynomials.size() + 1));

  for (size_t i = 0; i < polynomials.size(); i++) {
    PolynomialMatrix matrix(1, 1);
    matrix(0, 0) = polynomials[i];
    polynomials_.push_back(matrix);
  }
}

template <typename T>
std::unique_ptr<Trajectory<T>> PiecewisePolynomial<T>::Clone() const {
  return std::make_unique<PiecewisePolynomial<T>>(*this);
}

template <typename T>
PiecewisePolynomial<T>
PiecewisePolynomial<T>::derivative(int derivative_order) const {
  DRAKE_DEMAND(derivative_order >= 0);
  PiecewisePolynomial ret = *this;
  if (derivative_order == 0) {
    return ret;
  }
  for (auto it = ret.polynomials_.begin(); it != ret.polynomials_.end(); ++it) {
    PolynomialMatrix& matrix = *it;
    for (Eigen::Index row = 0; row < rows(); row++) {
      for (Eigen::Index col = 0; col < cols(); col++) {
        matrix(row, col) = matrix(row, col).Derivative(derivative_order);
      }
    }
  }
  return ret;
}

template <typename T>
PiecewisePolynomial<T>
PiecewisePolynomial<T>::integral(double value_at_start_time) const {
  CoefficientMatrix matrix_value_at_start_time =
      CoefficientMatrix::Constant(rows(), cols(), value_at_start_time);
  return integral(matrix_value_at_start_time);
}

template <typename T>
PiecewisePolynomial<T>
PiecewisePolynomial<T>::integral(
    const typename PiecewisePolynomial<T>::CoefficientMatrixRef&
        value_at_start_time) const {
  PiecewisePolynomial ret = *this;
  for (int segment_index = 0; segment_index < this->get_number_of_segments();
       segment_index++) {
    PolynomialMatrix& matrix = ret.polynomials_[segment_index];
    for (Eigen::Index row = 0; row < rows(); row++) {
      for (Eigen::Index col = 0; col < cols(); col++) {
        if (segment_index == 0) {
          matrix(row, col) =
              matrix(row, col).Integral(value_at_start_time(row, col));
        } else {
          matrix(row, col) =
              matrix(row, col).Integral(ret.segmentValueAtGlobalAbscissa(
                  segment_index - 1, this->start_time(segment_index), row,
                  col));
        }
      }
    }
  }
  return ret;
}

template <typename T>
double PiecewisePolynomial<T>::scalarValue(double t,
                                           Eigen::Index row,
                                           Eigen::Index col) const {
  int segment_index = this->get_segment_index(t);
  return segmentValueAtGlobalAbscissa(segment_index, t, row, col);
}

template <typename T>
MatrixX<T>
PiecewisePolynomial<T>::value(double t) const {
  int segment_index = this->get_segment_index(t);
  t = std::min(std::max(t, this->start_time()), this->end_time());
  Eigen::Matrix<double, PolynomialMatrix::RowsAtCompileTime,
                PolynomialMatrix::ColsAtCompileTime>
      ret(rows(), cols());
  for (Eigen::Index row = 0; row < rows(); row++) {
    for (Eigen::Index col = 0; col < cols(); col++) {
      ret(row, col) = segmentValueAtGlobalAbscissa(segment_index, t, row, col);
    }
  }
  return ret;
}

template <typename T>
const typename PiecewisePolynomial<T>::PolynomialMatrix&
PiecewisePolynomial<T>::getPolynomialMatrix(
    int segment_index) const {
  return polynomials_[segment_index];
}

template <typename T>
const Polynomial<T>& PiecewisePolynomial<T>::getPolynomial(
    int segment_index, Eigen::Index row, Eigen::Index col) const {
  this->segment_number_range_check(segment_index);
  return polynomials_[segment_index](row, col);
}

template <typename T>
int PiecewisePolynomial<T>::getSegmentPolynomialDegree(
    int segment_index, Eigen::Index row, Eigen::Index col) const {
  this->segment_number_range_check(segment_index);
  return polynomials_[segment_index](row, col).GetDegree();
}

template <typename T>
PiecewisePolynomial<T>& PiecewisePolynomial<T>::
operator+=(const PiecewisePolynomial<T>& other) {
  if (!this->SegmentTimesEqual(other))
    throw runtime_error(
        "Addition not yet implemented when segment times are not equal");
  for (size_t i = 0; i < polynomials_.size(); i++)
    polynomials_[i] += other.polynomials_[i];
  return *this;
}

template <typename T>
PiecewisePolynomial<T>& PiecewisePolynomial<T>::
operator-=(const PiecewisePolynomial<T>& other) {
  if (!this->SegmentTimesEqual(other))
    throw runtime_error(
        "Subtraction not yet implemented when segment times are not equal");
  for (size_t i = 0; i < polynomials_.size(); i++)
    polynomials_[i] -= other.polynomials_[i];
  return *this;
}

template <typename T>
PiecewisePolynomial<T>& PiecewisePolynomial<T>::
operator*=(const PiecewisePolynomial<T>& other) {
  if (!this->SegmentTimesEqual(other))
    throw runtime_error(
        "Multiplication not yet implemented when segment times are not equal");
  for (size_t i = 0; i < polynomials_.size(); i++) {
    polynomials_[i].array() *= other.polynomials_[i].array();
  }
  return *this;
}

template <typename T>
PiecewisePolynomial<T>& PiecewisePolynomial<T>::
operator+=(const typename PiecewisePolynomial<
           T>::CoefficientMatrix& offset) {
  for (size_t i = 0; i < polynomials_.size(); i++)
    polynomials_[i] += offset.template cast<PolynomialType>();
  return *this;
}

template <typename T>
PiecewisePolynomial<T>& PiecewisePolynomial<T>::
operator-=(const typename PiecewisePolynomial<
           T>::CoefficientMatrix& offset) {
  for (size_t i = 0; i < polynomials_.size(); i++)
    polynomials_[i] -= offset.template cast<PolynomialType>();
  return *this;
}

template <typename T>
const PiecewisePolynomial<T>
PiecewisePolynomial<T>::operator+(
    const PiecewisePolynomial<T>& other) const {
  PiecewisePolynomial<T> ret = *this;
  ret += other;
  return ret;
}

template <typename T>
const PiecewisePolynomial<T>
PiecewisePolynomial<T>::operator-(
    const PiecewisePolynomial<T>& other) const {
  PiecewisePolynomial<T> ret = *this;
  ret -= other;
  return ret;
}

template <typename T>
const PiecewisePolynomial<T>
    PiecewisePolynomial<T>::operator*(
        const PiecewisePolynomial<T>& other) const {
  PiecewisePolynomial<T> ret = *this;
  ret *= other;
  return ret;
}

template <typename T>
const PiecewisePolynomial<T>
PiecewisePolynomial<T>::operator+(
    const typename PiecewisePolynomial<T>::CoefficientMatrix&
        offset) const {
  PiecewisePolynomial<T> ret = *this;
  ret += offset;
  return ret;
}

template <typename T>
const PiecewisePolynomial<T>
PiecewisePolynomial<T>::operator-(
    const typename PiecewisePolynomial<T>::CoefficientMatrix&
        offset) const {
  PiecewisePolynomial<T> ret = *this;
  ret -= offset;
  return ret;
}

template <typename T>
bool PiecewisePolynomial<T>::isApprox(
    const PiecewisePolynomial<T>& other, double tol) const {
  if (rows() != other.rows() || cols() != other.cols()) return false;

  if (!this->SegmentTimesEqual(other, tol)) return false;

  for (int segment_index = 0; segment_index < this->get_number_of_segments();
       segment_index++) {
    const PolynomialMatrix& matrix = polynomials_[segment_index];
    const PolynomialMatrix& other_matrix = other.polynomials_[segment_index];
    for (Eigen::Index row = 0; row < rows(); row++) {
      for (Eigen::Index col = 0; col < cols(); col++) {
        if (!matrix(row, col).IsApprox(other_matrix(row, col), tol))
          return false;
      }
    }
  }
  return true;
}

template <typename T>
void PiecewisePolynomial<T>::ConcatenateInTime(
    const PiecewisePolynomial<T>& other) {
  if (!empty()) {
    // Performs basic sanity checks.
    DRAKE_THROW_UNLESS(this->rows() == other.rows());
    DRAKE_THROW_UNLESS(this->cols() == other.cols());
    const double time_offset = other.start_time() - this->end_time();
    // Absolute tolerance is scaled along with the time scale.
    const double absolute_tolerance = std::max(std::abs(this->end_time()), 1.) *
                                      std::numeric_limits<double>::epsilon();
    DRAKE_THROW_UNLESS(std::abs(time_offset) < absolute_tolerance);
    // Gets instance breaks.
    std::vector<double>& breaks = this->get_mutable_breaks();
    // Drops first break to avoid duplication.
    breaks.pop_back();
    // Concatenates other breaks, while shifting them appropriately
    // for both trajectories to be time-aligned.
    for (double other_break : other.breaks()) {
      breaks.push_back(other_break - time_offset);
    }
    // Concatenates other polynomials.
    polynomials_.insert(polynomials_.end(),
                        other.polynomials_.begin(),
                        other.polynomials_.end());
  } else {
    std::vector<double>& breaks = this->get_mutable_breaks();
    breaks = other.breaks();
    polynomials_ = other.polynomials_;
  }
}

template <typename T>
void PiecewisePolynomial<T>::shiftRight(double offset) {
  std::vector<double>& breaks = this->get_mutable_breaks();
  for (auto it = breaks.begin(); it != breaks.end(); ++it) {
    *it += offset;
  }
}

template <typename T>
void PiecewisePolynomial<T>::setPolynomialMatrixBlock(
    const typename PiecewisePolynomial<T>::PolynomialMatrix&
        replacement,
    int segment_number, Eigen::Index row_start, Eigen::Index col_start) {
  this->segment_number_range_check(segment_number);
  polynomials_[segment_number].block(row_start, col_start, replacement.rows(),
                                     replacement.cols()) = replacement;
}

template <typename T>
PiecewisePolynomial<T>
PiecewisePolynomial<T>::slice(int start_segment_index, int num_segments) const {
  this->segment_number_range_check(start_segment_index);
  this->segment_number_range_check(start_segment_index + num_segments - 1);

  auto breaks_start_it = this->breaks().begin() + start_segment_index;
  auto breaks_slice = vector<double>(
      breaks_start_it,
      breaks_start_it + num_segments +
          1);  // + 1 because there's one more segment times than segments.

  auto polynomials_start_it = polynomials_.begin() + start_segment_index;
  auto polynomials_slice = vector<PolynomialMatrix>(
      polynomials_start_it, polynomials_start_it + num_segments);

  return PiecewisePolynomial<T>(polynomials_slice,
                                              breaks_slice);
}

template <typename T>
double PiecewisePolynomial<T>::segmentValueAtGlobalAbscissa(
    int segment_index, double t, Eigen::Index row, Eigen::Index col) const {
  return polynomials_[segment_index](row, col).EvaluateUnivariate(
      t - this->start_time(segment_index));
}

template <typename T>
Eigen::Index PiecewisePolynomial<T>::rows() const {
  if (polynomials_.size() > 0) {
    return polynomials_[0].rows();
  } else {
    throw std::runtime_error(
        "PiecewisePolynomial has no segments. Number of rows is undefined.");
  }
}

template <typename T>
Eigen::Index PiecewisePolynomial<T>::cols() const {
  if (polynomials_.size() > 0) {
    return polynomials_[0].cols();
  } else {
    throw std::runtime_error(
        "PiecewisePolynomial has no segments. Number of columns is undefined.");
  }
}

// Static generators for splines.

// Throws std::runtime_error if these conditions are true:
//  `breaks` and `knots` have different length,
//  `knots` have inconsistent dimensions,
//  any `knots` have either 0 rows or 0 cols,
//  `breaks` is not strictly increasing,
//  `breaks` has length smaller than `min_length`.
template <typename T>
void PiecewisePolynomial<T>::
    CheckSplineGenerationInputValidityOrThrow(
        const std::vector<double>& breaks,
        const std::vector<CoefficientMatrix>& knots,
        int min_length) {
  const std::vector<double>& times = breaks;
  const std::vector<CoefficientMatrix>& Y = knots;
  if (times.size() != Y.size()) {
    throw std::runtime_error(
        "Number of break points does not match number of knots.");
  }
  if (static_cast<int>(times.size()) < min_length) {
    throw std::runtime_error("Not enough knots.");
  }
  Eigen::Index rows = Y.front().rows();
  Eigen::Index cols = Y.front().cols();
  if (rows < 1 || cols < 1) {
    throw std::runtime_error("Knots need to be non-empty.");
  }
  for (const auto& y : Y) {
    if (y.rows() != rows || y.cols() != cols) {
      throw std::runtime_error("Knots have inconsistent dimensions.");
    }
  }
  for (size_t i = 0; i < times.size() - 1; i++) {
    if (times[i + 1] - times[i] < PiecewiseTrajectory<T>::kEpsilonTime) {
      throw std::runtime_error("times must be in increasing order.");
    }
  }
}

// Makes a piecewise constant polynomial.
// The value for each segment is set to the value at the beginning of each
// segment.
template <typename T>
PiecewisePolynomial<T>
PiecewisePolynomial<T>::ZeroOrderHold(
    const std::vector<double>& breaks,
    const std::vector<CoefficientMatrix>& knots) {
  CheckSplineGenerationInputValidityOrThrow(breaks, knots, 2);

  std::vector<PolynomialMatrix> polys;
  polys.reserve(breaks.size() - 1);
  // For each of the breaks, creates a PolynomialMatrix which can contain joint
  // positions.
  for (int i = 0; i < static_cast<int>(breaks.size()) - 1; ++i) {
    PolynomialMatrix poly_matrix(knots[0].rows(), knots[0].cols());

    for (int j = 0; j < knots[i].rows(); ++j) {
      for (int k = 0; k < knots[i].cols(); ++k) {
        poly_matrix(j, k) = PolynomialType(
            Eigen::Matrix<T, 1, 1>(knots[i](j, k)));
      }
    }
    polys.push_back(poly_matrix);
  }
  return PiecewisePolynomial<T>(polys, breaks);
}

// Makes a piecewise linear polynomial.
template <typename T>
PiecewisePolynomial<T>
PiecewisePolynomial<T>::FirstOrderHold(
    const std::vector<double>& breaks,
    const std::vector<CoefficientMatrix>& knots) {
  CheckSplineGenerationInputValidityOrThrow(breaks, knots, 2);

  std::vector<PolynomialMatrix> polys;
  polys.reserve(breaks.size() - 1);
  // For each of the breaks, creates a PolynomialMatrix which can contain joint
  // positions.
  for (int i = 0; i < static_cast<int>(breaks.size()) - 1; ++i) {
    PolynomialMatrix poly_matrix(knots[0].rows(), knots[0].cols());

    for (int j = 0; j < knots[i].rows(); ++j) {
      for (int k = 0; k < knots[i].cols(); ++k) {
        poly_matrix(j, k) = PolynomialType(Eigen::Matrix<T, 2, 1>(
            knots[i](j, k), (knots[i + 1](j, k) - knots[i](j, k)) /
                                (breaks[i + 1] - breaks[i])));
      }
    }
    polys.push_back(poly_matrix);
  }
  return PiecewisePolynomial<T>(polys, breaks);
}

template <typename T>
static int sign(T val, T tol) {
  if (val < -tol)
    return -1;
  else if (val > tol)
    return 1;
  return 0;
}

// Computes the first derivative for either the starting or the end knot point.
// This is an internal helpful function for pchip.
// The first derivative is computed using a non-centered, shape-preserving
// three-point formulae.
// See equation (2.10) in the following reference for more details.
// http://www.mi.sanu.ac.rs/~gvm/radovi/mon.pdf
template <typename T>
MatrixX<T>
PiecewisePolynomial<T>::ComputePchipEndSlope(
    double dt0, double dt1, const CoefficientMatrix& slope0,
    const CoefficientMatrix& slope1) {
  CoefficientMatrix deriv =
      ((2.0 * dt0 + dt1) * slope0 - dt0 * slope1) / (dt0 + dt1);
  for (int i = 0; i < deriv.rows(); ++i) {
    for (int j = 0; j < deriv.cols(); ++j) {
      if (sign(deriv(i, j), kSlopeEpsilon) !=
          sign(slope0(i, j), kSlopeEpsilon)) {
        deriv(i, j) = 0.;
      } else if (sign(slope0(i, j), kSlopeEpsilon) !=
                 sign(slope1(i, j), kSlopeEpsilon) &&
                 std::abs(deriv(i, j)) > std::abs(3. * slope0(i, j))) {
        deriv(i, j) = 3. * slope0(i, j);
      }
    }
  }
  return deriv;
}

// Makes a cubic piecewise polynomial.
// It first computes the first derivatives at each break, and solves for each
// segment's coefficients using the derivatives and knots.
// The derivatives are computed using a weighted harmonic mean for internal
// points, and ComputePchipEndSlope is used for computing the end points'
// derivatives.
// See pg 9 in http://home.uchicago.edu/~sctchoi/courses/cs138/interp.pdf for
// more details.
template <typename T>
PiecewisePolynomial<T>
PiecewisePolynomial<T>::Pchip(
    const std::vector<double>& breaks,
    const std::vector<CoefficientMatrix>& knots,
    bool zero_end_point_derivatives) {
  const std::vector<double>& times = breaks;
  const std::vector<CoefficientMatrix>& Y = knots;

  if (zero_end_point_derivatives) {
    CheckSplineGenerationInputValidityOrThrow(times, Y, 2);
  } else {
    CheckSplineGenerationInputValidityOrThrow(times, Y, 3);
  }

  int N = static_cast<int>(times.size());
  int rows = Y.front().rows();
  int cols = Y.front().cols();

  std::vector<PolynomialMatrix> polynomials(N - 1);
  std::vector<CoefficientMatrix> slope(N - 1);
  std::vector<double> dt(N - 1);

  std::vector<CoefficientMatrix> Ydot(N, CoefficientMatrix::Zero(rows, cols));
  Eigen::Matrix<T, 4, 1> coeffs;

  // Computes the end slopes.
  CoefficientMatrix Ydot_start = CoefficientMatrix::Zero(rows, cols);
  CoefficientMatrix Ydot_end = CoefficientMatrix::Zero(rows, cols);

  if (!zero_end_point_derivatives) {
    Ydot_start = ComputePchipEndSlope(times[1] - times[0], times[2] - times[1],
                                      (Y[1] - Y[0]) / (times[1] - times[0]),
                                      (Y[2] - Y[1]) / (times[2] - times[1]));
    Ydot_end = ComputePchipEndSlope(
        times[N - 1] - times[N - 2], times[N - 2] - times[N - 3],
        (Y[N - 1] - Y[N - 2]) / (times[N - 1] - times[N - 2]),
        (Y[N - 2] - Y[N - 3]) / (times[N - 2] - times[N - 3]));
  }

  for (int t = 0; t < N - 1; ++t) {
    dt[t] = times[t + 1] - times[t];
    slope[t] = (Y[t + 1] - Y[t]) / dt[t];
    polynomials[t].resize(Y[t].rows(), Y[t].cols());
  }

  for (int j = 0; j < rows; ++j) {
    for (int k = 0; k < cols; ++k) {
      // Computes Ydot.
      for (size_t t = 0; t < dt.size() - 1; ++t) {
        // knot[t+1] is local extrema.
        if (slope[t](j, k) * slope[t + 1](j, k) <= 0) {
          Ydot[t + 1](j, k) = 0;
        } else {
          // Computed with using weighted harmonic mean.
          T common = dt[t] + dt[t + 1];
          Ydot[t + 1](j, k) =
              3 * common / ((common + dt[t + 1]) / slope[t](j, k) +
                            (common + dt[t]) / slope[t + 1](j, k));
        }
      }

      // Fixes end point slopes.
      Ydot[0](j, k) = Ydot_start(j, k);
      Ydot[N - 1](j, k) = Ydot_end(j, k);

      // Computes coeffs given Y and Ydot at the end points for each segment.
      for (int t = 0; t < N - 1; ++t) {
        coeffs = ComputeCubicSplineCoeffs(dt[t], Y[t](j, k), Y[t + 1](j, k),
                                          Ydot[t](j, k), Ydot[t + 1](j, k));
        polynomials[t](j, k) = PolynomialType(coeffs);
      }
    }
  }

  return PiecewisePolynomial<T>(polynomials, times);
}

// Makes a cubic piecewise polynomial using the given knots and their
// derivatives at each break.
template <typename T>
PiecewisePolynomial<T>
PiecewisePolynomial<T>::Cubic(
    const std::vector<double>& breaks,
    const std::vector<CoefficientMatrix>& knots,
    const std::vector<CoefficientMatrix>& knots_dot) {
  const std::vector<double>& times = breaks;
  const std::vector<CoefficientMatrix>& Y = knots;
  const std::vector<CoefficientMatrix>& Ydot = knots_dot;
  CheckSplineGenerationInputValidityOrThrow(times, Y, 2);

  int N = static_cast<int>(times.size());
  int rows = Y.front().rows();
  int cols = Y.front().cols();

  if (times.size() != Ydot.size()) {
    throw std::runtime_error("Y and Ydot have different length.");
  }
  for (int t = 0; t < N; ++t) {
    if (rows != Ydot[t].rows() || cols != Ydot[t].cols()) {
      throw std::runtime_error("Y and Ydot dimension mismatch.");
    }
  }

  std::vector<PolynomialMatrix> polynomials(N - 1);

  for (int t = 0; t < N - 1; ++t) {
    polynomials[t].resize(Y[t].rows(), Y[t].cols());
    for (int i = 0; i < rows; ++i) {
      for (int j = 0; j < cols; ++j) {
        double dt = times[t + 1] - times[t];
        Eigen::Matrix<T, 4, 1> coeffs = ComputeCubicSplineCoeffs(
            dt, Y[t](i, j), Y[t + 1](i, j), Ydot[t](i, j), Ydot[t + 1](i, j));
        polynomials[t](i, j) = PolynomialType(coeffs);
      }
    }
  }

  return PiecewisePolynomial<T>(polynomials, times);
}

// Sets up the linear system for solving for the cubic piecewise polynomial
// coefficients.
// See the header file for more information.
template <typename T>
int PiecewisePolynomial<T>::
    SetupCubicSplineInteriorCoeffsLinearSystem(
        const std::vector<double>& breaks,
        const std::vector<CoefficientMatrix>& knots,
        int row, int col,
        MatrixX<T>* A,
        VectorX<T>* b) {
  const std::vector<double>& times = breaks;
  const std::vector<CoefficientMatrix>& Y = knots;
  int N = static_cast<int>(times.size());

  DRAKE_DEMAND(A != nullptr);
  DRAKE_DEMAND(b != nullptr);
  DRAKE_DEMAND(A->rows() == 4 * (N - 1));
  DRAKE_DEMAND(A->cols() == 4 * (N - 1));
  DRAKE_DEMAND(b->rows() == 4 * (N - 1));

  int row_idx = 0;
  MatrixX<T>& Aref = *A;
  VectorX<T>& bref = *b;

  for (int i = 0; i < N - 1; ++i) {
    double dt = times[i + 1] - times[i];

    // y_i(x_i) = a0i = Y[i]
    Aref(row_idx, 4 * i) = 1;
    bref(row_idx++) = Y[i](row, col);

    // y_i(x_{i+1}) = y_{i+1}(x_{i}) =>
    // a0i + a1i*(x_{i+1} - x_i) + a2i(x_{i+1} - x_i)^2 + a3i(x_{i+1} -
    // x_i)^3 = a0{i+1}
    Aref(row_idx, 4 * i + 0) = 1;
    Aref(row_idx, 4 * i + 1) = dt;
    Aref(row_idx, 4 * i + 2) = dt * dt;
    Aref(row_idx, 4 * i + 3) = dt * dt * dt;
    if (i != N - 2) {
      Aref(row_idx++, 4 * (i + 1)) = -1;
    } else {
      bref(row_idx++) = Y[N - 1](row, col);
    }

    // y_i'(x_{i+1}) = y_{i+1}'(x_{i}) =>
    // a1i + 2*a2i(x_{i+1} - x_i) + 3*a3i(x_{i+1} - x_i)^2 = a1{i+1}
    if (i != N - 2) {
      Aref(row_idx, 4 * i + 1) = 1;
      Aref(row_idx, 4 * i + 2) = 2 * dt;
      Aref(row_idx, 4 * i + 3) = 3 * dt * dt;
      Aref(row_idx++, 4 * (i + 1) + 1) = -1;
    }

    if (i != N - 2) {
      // y_i''(x_{i+1}) = y_{i+1}''(x_{i}) =>
      // 2*a2i + 6*a3i(x_{i+1} - x_i) = 2*a2{i+1}
      Aref(row_idx, 4 * i + 2) = 2;
      Aref(row_idx, 4 * i + 3) = 6 * dt;
      Aref(row_idx++, 4 * (i + 1) + 2) = -2;
    }
  }
  DRAKE_DEMAND(row_idx == 4 * (N - 1) - 2);
  return row_idx;
}

// Makes a cubic piecewise polynomial.
// Internal knot points have continuous values, first and second derivatives,
// and first derivatives at both end points are set to `knot_dot_at_start`
// and `knot_dot_at_end`.
template <typename T>
PiecewisePolynomial<T>
PiecewisePolynomial<T>::Cubic(
    const std::vector<double>& breaks,
    const std::vector<CoefficientMatrix>& knots,
    const CoefficientMatrix& knot_dot_at_start,
    const CoefficientMatrix& knot_dot_at_end) {
  const std::vector<double>& times = breaks;
  const std::vector<CoefficientMatrix>& Y = knots;
  const CoefficientMatrix& Ydot_start = knot_dot_at_start;
  const CoefficientMatrix& Ydot_end = knot_dot_at_end;

  CheckSplineGenerationInputValidityOrThrow(times, Y, 2);

  int N = static_cast<int>(times.size());
  int rows = Y.front().rows();
  int cols = Y.front().cols();

  if (Ydot_start.rows() != rows || Ydot_start.cols() != cols) {
    throw std::runtime_error("Ydot_start and Y dimension mismatch");
  }

  if (Ydot_end.rows() != rows || Ydot_end.cols() != cols) {
    throw std::runtime_error("Ydot_end and Y dimension mismatch");
  }

  std::vector<PolynomialMatrix> polynomials(N - 1);
  for (int i = 0; i < N - 1; ++i) {
    polynomials[i].resize(rows, cols);
  }

  MatrixX<T> A(4 * (N - 1), 4 * (N - 1));
  VectorX<T> b(4 * (N - 1));
  VectorX<T> solution;

  A.setZero();
  b.setZero();

  // Sets up a linear equation to solve for the coefficients.
  for (int j = 0; j < rows; ++j) {
    for (int k = 0; k < cols; ++k) {
      int row_idx =
          SetupCubicSplineInteriorCoeffsLinearSystem(times, Y, j, k, &A, &b);

      // Endpoints' velocity matches the given ones.
      A(row_idx, 1) = 1;
      b(row_idx++) = Ydot_start(j, k);

      A(row_idx, 4 * (N - 2) + 1) = 1;
      A(row_idx, 4 * (N - 2) + 2) = 2 * (times[N - 1] - times[N - 2]);
      A(row_idx, 4 * (N - 2) + 3) =
          3 * (times[N - 1] - times[N - 2]) * (times[N - 1] - times[N - 2]);
      b(row_idx++) = Ydot_end(j, k);

      // TODO(siyuan.feng): Should switch to a sparse solver.
      solution = A.colPivHouseholderQr().solve(b);

      for (int i = 0; i < N - 1; ++i) {
        polynomials[i](j, k) =
            Polynomial<T>(solution.template segment<4>(4 * i));
      }
    }
  }

  return PiecewisePolynomial<T>(polynomials, times);
}

// Makes a cubic piecewise polynomial.
// Internal knot points have continuous values, first and second derivatives.
// If `periodic_end_condition` is `true`, the first and second derivatives will
// be continuous between the end of the last segment and the beginning of
// the first. Otherwise, the third derivative is made continuous between the
// first two segments and between the last two segments (the "not-a-knot"
// end condition).
template <typename T>
PiecewisePolynomial<T>
PiecewisePolynomial<T>::Cubic(
    const std::vector<double>& breaks,
    const std::vector<CoefficientMatrix>& knots,
    bool periodic_end_condition) {
  const std::vector<double>& times = breaks;
  const std::vector<CoefficientMatrix>& Y = knots;
  CheckSplineGenerationInputValidityOrThrow(times, Y, 3);

  int N = static_cast<int>(times.size());
  int rows = Y.front().rows();
  int cols = Y.front().cols();

  std::vector<PolynomialMatrix> polynomials(N - 1);
  for (int i = 0; i < N - 1; ++i) {
    polynomials[i].resize(rows, cols);
  }

  MatrixX<T> A(4 * (N - 1), 4 * (N - 1));
  VectorX<T> b(4 * (N - 1));
  VectorX<T> solution;

  A.setZero();
  b.setZero();

  // Sets up a linear equation to solve for the coefficients.
  for (int j = 0; j < rows; ++j) {
    for (int k = 0; k < cols; ++k) {
      int row_idx =
          SetupCubicSplineInteriorCoeffsLinearSystem(times, Y, j, k, &A, &b);

      if (periodic_end_condition) {
        // Time during the last segment.
        const double end_dt = times[times.size() - 1] - times[times.size() - 2];
        // Enforce velocity between end-of-last and beginning-of-first segments
        // is continuous.
        A(row_idx, 1) = -1;  // Linear term of 1st segment.
        A(row_idx, 4 * (N - 2) + 1) = 1;  // Linear term of last segment.
        A(row_idx, 4 * (N - 2) + 2) = 2 * end_dt;  // Squared term of last
                                                   // segment.
        A(row_idx, 4 * (N - 2) + 3) = 3 * end_dt * end_dt;  // Cubic term of
                                                            // last segment.
        b(row_idx++) = 0;

        // Enforce that acceleration between end-of-last and beginning-of-first
        // segments is continuous.
        A(row_idx, 2) = -2;  // Quadratic term of 1st segment.
        A(row_idx, 4 * (N - 2) + 2) = 2;  // Quadratic term of last segment.
        A(row_idx, 4 * (N - 2) + 3) = 6 * end_dt;  // Cubic term of last
                                                   // segment.
        b(row_idx++) = 0;
      } else {
        if (N > 3) {
          // Ydddot(times[1]) is continuous.
          A(row_idx, 3) = 1;  // Cubic term of 1st segment.
          A(row_idx, 4 + 3) = -1;  // Cubic term of 2nd segment.
          b(row_idx++) = 0;

          // Ydddot(times[N-2]) is continuous.
          A(row_idx, 4 * (N - 3) + 3) = 1;
          A(row_idx, 4 * (N - 2) + 3) = -1;
          b(row_idx++) = 0;
        } else {
          // Set Jerk to zero if only have 3 points, becomes a quadratic.
          A(row_idx, 3) = 1;
          b(row_idx++) = 0;

          A(row_idx, 4 + 3) = 1;
          b(row_idx++) = 0;
        }
      }

      // TODO(siyuan.feng): Should switch to a sparse solver.
      solution = A.colPivHouseholderQr().solve(b);

      for (int i = 0; i < N - 1; ++i) {
        polynomials[i](j, k) =
            Polynomial<T>(solution.template segment<4>(4 * i));
      }
    }
  }

  return PiecewisePolynomial<T>(polynomials, times);
}

namespace {

// Helper method to go from the Eigen entry points to the std::vector versions.
// Copies each column of mat to an element of the returned std::vector.
template <typename T>
std::vector<MatrixX<T>> ColsToStdVector(
    const Eigen::Ref<const MatrixX<T>>& mat) {
  std::vector<MatrixX<T>> vec(mat.cols());
  for (int i=0; i < mat.cols(); i++) {
    vec[i] = mat.col(i);
  }
  return vec;
}

}  // end namespace

template <typename T>
PiecewisePolynomial<T> PiecewisePolynomial<T>::ZeroOrderHold(
    const Eigen::Ref<const Eigen::VectorXd>& breaks,
    const Eigen::Ref<const MatrixX<T>>& knots) {
  DRAKE_DEMAND(knots.cols() == breaks.size());
  std::vector<double> my_breaks(breaks.data(), breaks.data() + breaks.size());
  return PiecewisePolynomial<T>::ZeroOrderHold(my_breaks,
                                               ColsToStdVector(knots));
}

template <typename T>
PiecewisePolynomial<T> PiecewisePolynomial<T>::FirstOrderHold(
    const Eigen::Ref<const Eigen::VectorXd>& breaks,
    const Eigen::Ref<const MatrixX<T>>& knots) {
  DRAKE_DEMAND(knots.cols() == breaks.size());
  std::vector<double> my_breaks(breaks.data(), breaks.data() + breaks.size());
  return PiecewisePolynomial<T>::FirstOrderHold(my_breaks,
                                                ColsToStdVector(knots));
}

template <typename T>
PiecewisePolynomial<T> PiecewisePolynomial<T>::Pchip(
    const Eigen::Ref<const Eigen::VectorXd>& breaks,
    const Eigen::Ref<const MatrixX<T>>& knots,
    bool zero_end_point_derivatives) {
  DRAKE_DEMAND(knots.cols() == breaks.size());
  std::vector<double> my_breaks(breaks.data(), breaks.data() + breaks.size());
  return PiecewisePolynomial<T>::Pchip(
      my_breaks, ColsToStdVector(knots), zero_end_point_derivatives);
}

template <typename T>
PiecewisePolynomial<T> PiecewisePolynomial<T>::Cubic(
    const Eigen::Ref<const Eigen::VectorXd>& breaks,
    const Eigen::Ref<const MatrixX<T>>& knots,
    const Eigen::Ref<const VectorX<T>>& knots_dot_start,
    const Eigen::Ref<const VectorX<T>>& knots_dot_end) {
  DRAKE_DEMAND(knots.cols() == breaks.size());
  std::vector<double> my_breaks(breaks.data(), breaks.data() + breaks.size());
  return PiecewisePolynomial<T>::Cubic(my_breaks, ColsToStdVector(knots),
                                       knots_dot_start.eval(),
                                       knots_dot_end.eval());
}

template <typename T>
PiecewisePolynomial<T> PiecewisePolynomial<T>::Cubic(
    const Eigen::Ref<const Eigen::VectorXd>& breaks,
    const Eigen::Ref<const MatrixX<T>>& knots,
    const Eigen::Ref<const MatrixX<T>>& knots_dot) {
  DRAKE_DEMAND(knots.cols() == breaks.size());
  std::vector<double> my_breaks(breaks.data(), breaks.data() + breaks.size());
  return PiecewisePolynomial<T>::Cubic(my_breaks, ColsToStdVector(knots),
                                       ColsToStdVector(knots_dot));
}

template <typename T>
PiecewisePolynomial<T> PiecewisePolynomial<T>::Cubic(
    const Eigen::Ref<const Eigen::VectorXd>& breaks,
    const Eigen::Ref<const MatrixX<T>>& knots,
    bool periodic_end_condition) {
  DRAKE_DEMAND(knots.cols() == breaks.size());
  std::vector<double> my_breaks(breaks.data(), breaks.data() + breaks.size());
  return PiecewisePolynomial<T>::Cubic(my_breaks, ColsToStdVector(knots),
                                       periodic_end_condition);
}

// Computes the cubic spline coefficients based on the given values and first
// derivatives at both end points.
template <typename T>
Eigen::Matrix<T, 4, 1> PiecewisePolynomial<T>::ComputeCubicSplineCoeffs(
    double dt, T y0, T y1, T yd0, T yd1) {
  if (dt < PiecewiseTrajectory<T>::kEpsilonTime) {
    throw std::runtime_error("dt < epsilon.");
  }

  double dt2 = dt * dt;
  T c4 = y0;
  T c3 = yd0;
  T common = (yd1 - c3 - 2. / dt * (y1 - c4 - dt * c3));
  T c1 = 1. / dt2 * common;
  T c2 = 1. / dt2 * (y1 - c4 - dt * c3 - dt * common);
  return Vector4<T>(c4, c3, c2, c1);
}

// Explicit instantiations.
template class PiecewisePolynomial<double>;
// doesn't work yet
// template class PiecewisePolynomial<std::complex<double>>;

}  // namespace trajectories
}  // namespace drake

