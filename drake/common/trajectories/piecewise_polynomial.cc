#include "drake/common/trajectories/piecewise_polynomial.h"

#include <algorithm>

#include "drake/common/drake_assert.h"

using std::runtime_error;
using std::vector;

template <typename CoefficientType>
PiecewisePolynomial<CoefficientType>::PiecewisePolynomial(
    std::vector<PolynomialMatrix> const& polynomials,
    std::vector<double> const& breaks)
    : PiecewisePolynomialBase(breaks), polynomials_(polynomials) {
  DRAKE_ASSERT(breaks.size() == (polynomials.size() + 1));
  for (int i = 1; i < getNumberOfSegments(); i++) {
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

template <typename CoefficientType>
PiecewisePolynomial<CoefficientType>::PiecewisePolynomial(
    std::vector<PolynomialType> const& polynomials,
    std::vector<double> const& breaks)
    : PiecewisePolynomialBase(breaks) {
  DRAKE_ASSERT(breaks.size() == (polynomials.size() + 1));

  for (size_t i = 0; i < polynomials.size(); i++) {
    PolynomialMatrix matrix(1, 1);
    matrix(0, 0) = polynomials[i];
    polynomials_.push_back(matrix);
  }
}

template <typename CoefficientType>
PiecewisePolynomial<CoefficientType>::PiecewisePolynomial() {
  // empty
}

template <typename CoefficientType>
PiecewisePolynomial<CoefficientType>
PiecewisePolynomial<CoefficientType>::derivative(int derivative_order) const {
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

template <typename CoefficientType>
PiecewisePolynomial<CoefficientType> PiecewisePolynomial<
    CoefficientType>::integral(double value_at_start_time) const {
  CoefficientMatrix matrix_value_at_start_time =
      CoefficientMatrix::Constant(rows(), cols(), value_at_start_time);
  return integral(matrix_value_at_start_time);
}

template <typename CoefficientType>
PiecewisePolynomial<CoefficientType>
PiecewisePolynomial<CoefficientType>::integral(
    const typename PiecewisePolynomial<CoefficientType>::CoefficientMatrixRef&
        value_at_start_time) const {
  PiecewisePolynomial ret = *this;
  for (int segment_index = 0; segment_index < getNumberOfSegments();
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
                  segment_index - 1, getStartTime(segment_index), row, col));
        }
      }
    }
  }
  return ret;
}

template <typename CoefficientType>
double PiecewisePolynomial<CoefficientType>::scalarValue(double t,
                                                         Eigen::Index row,
                                                         Eigen::Index col) {
  int segment_index = getSegmentIndex(t);
  return segmentValueAtGlobalAbscissa(segment_index, t, row, col);
}

template <typename CoefficientType>
drake::MatrixX<double>
PiecewisePolynomial<CoefficientType>::value(double t) const {
  int segment_index = getSegmentIndex(t);
  t = std::min(std::max(t, getStartTime()), getEndTime());
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

template <typename CoefficientType>
const typename PiecewisePolynomial<CoefficientType>::PolynomialMatrix&
PiecewisePolynomial<CoefficientType>::getPolynomialMatrix(
    int segment_index) const {
  return polynomials_[segment_index];
}

template <typename CoefficientType>
const Polynomial<CoefficientType>&
PiecewisePolynomial<CoefficientType>::getPolynomial(int segment_index,
                                                    Eigen::Index row,
                                                    Eigen::Index col) const {
  segmentNumberRangeCheck(segment_index);
  return polynomials_[segment_index](row, col);
}

template <typename CoefficientType>
int PiecewisePolynomial<CoefficientType>::getSegmentPolynomialDegree(
    int segment_index, Eigen::Index row, Eigen::Index col) const {
  segmentNumberRangeCheck(segment_index);
  return polynomials_[segment_index](row, col).GetDegree();
}

template <typename CoefficientType>
PiecewisePolynomial<CoefficientType>& PiecewisePolynomial<CoefficientType>::
operator+=(const PiecewisePolynomial<CoefficientType>& other) {
  if (!segmentTimesEqual(other, kEpsilonTime))
    throw runtime_error(
        "Addition not yet implemented when segment times are not equal");
  for (size_t i = 0; i < polynomials_.size(); i++)
    polynomials_[i] += other.polynomials_[i];
  return *this;
}

template <typename CoefficientType>
PiecewisePolynomial<CoefficientType>& PiecewisePolynomial<CoefficientType>::
operator-=(const PiecewisePolynomial<CoefficientType>& other) {
  if (!segmentTimesEqual(other, kEpsilonTime))
    throw runtime_error(
        "Addition not yet implemented when segment times are not equal");
  for (size_t i = 0; i < polynomials_.size(); i++)
    polynomials_[i] -= other.polynomials_[i];
  return *this;
}

template <typename CoefficientType>
PiecewisePolynomial<CoefficientType>& PiecewisePolynomial<CoefficientType>::
operator*=(const PiecewisePolynomial<CoefficientType>& other) {
  if (!segmentTimesEqual(other, kEpsilonTime))
    throw runtime_error(
        "Multiplication not yet implemented when segment times are not equal");
  for (size_t i = 0; i < polynomials_.size(); i++) {
    polynomials_[i].array() *= other.polynomials_[i].array();
  }
  return *this;
}

template <typename CoefficientType>
PiecewisePolynomial<CoefficientType>& PiecewisePolynomial<CoefficientType>::
operator+=(const typename PiecewisePolynomial<
           CoefficientType>::CoefficientMatrix& offset) {
  for (size_t i = 0; i < polynomials_.size(); i++)
    polynomials_[i] += offset.template cast<PolynomialType>();
  return *this;
}

template <typename CoefficientType>
PiecewisePolynomial<CoefficientType>& PiecewisePolynomial<CoefficientType>::
operator-=(const typename PiecewisePolynomial<
           CoefficientType>::CoefficientMatrix& offset) {
  for (size_t i = 0; i < polynomials_.size(); i++)
    polynomials_[i] -= offset.template cast<PolynomialType>();
  return *this;
}

template <typename CoefficientType>
const PiecewisePolynomial<CoefficientType>
PiecewisePolynomial<CoefficientType>::operator+(
    const PiecewisePolynomial<CoefficientType>& other) const {
  PiecewisePolynomial<CoefficientType> ret = *this;
  ret += other;
  return ret;
}

template <typename CoefficientType>
const PiecewisePolynomial<CoefficientType>
PiecewisePolynomial<CoefficientType>::operator-(
    const PiecewisePolynomial<CoefficientType>& other) const {
  PiecewisePolynomial<CoefficientType> ret = *this;
  ret -= other;
  return ret;
}

template <typename CoefficientType>
const PiecewisePolynomial<CoefficientType>
    PiecewisePolynomial<CoefficientType>::operator*(
        const PiecewisePolynomial<CoefficientType>& other) const {
  PiecewisePolynomial<CoefficientType> ret = *this;
  ret *= other;
  return ret;
}

template <typename CoefficientType>
const PiecewisePolynomial<CoefficientType>
PiecewisePolynomial<CoefficientType>::operator+(
    const typename PiecewisePolynomial<CoefficientType>::CoefficientMatrix&
        offset) const {
  PiecewisePolynomial<CoefficientType> ret = *this;
  ret += offset;
  return ret;
}

template <typename CoefficientType>
const PiecewisePolynomial<CoefficientType>
PiecewisePolynomial<CoefficientType>::operator-(
    const typename PiecewisePolynomial<CoefficientType>::CoefficientMatrix&
        offset) const {
  PiecewisePolynomial<CoefficientType> ret = *this;
  ret -= offset;
  return ret;
}

template <typename CoefficientType>
bool PiecewisePolynomial<CoefficientType>::isApprox(
    const PiecewisePolynomial<CoefficientType>& other, double tol) const {
  if (rows() != other.rows() || cols() != other.cols()) return false;

  if (!segmentTimesEqual(other, tol)) return false;

  for (int segment_index = 0; segment_index < getNumberOfSegments();
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

template <typename CoefficientType>
void PiecewisePolynomial<CoefficientType>::shiftRight(double offset) {
  for (auto it = breaks.begin(); it != breaks.end(); ++it) {
    *it += offset;
  }
}

template <typename CoefficientType>
void PiecewisePolynomial<CoefficientType>::setPolynomialMatrixBlock(
    const typename PiecewisePolynomial<CoefficientType>::PolynomialMatrix&
        replacement,
    int segment_number, Eigen::Index row_start, Eigen::Index col_start) {
  segmentNumberRangeCheck(segment_number);
  polynomials_[segment_number].block(row_start, col_start, replacement.rows(),
                                     replacement.cols()) = replacement;
}

template <typename CoefficientType>
PiecewisePolynomial<CoefficientType>
PiecewisePolynomial<CoefficientType>::slice(int start_segment_index,
                                            int num_segments) const {
  segmentNumberRangeCheck(start_segment_index);
  segmentNumberRangeCheck(start_segment_index + num_segments - 1);

  auto breaks_start_it = breaks.begin() + start_segment_index;
  auto breaks_slice = vector<double>(
      breaks_start_it,
      breaks_start_it + num_segments +
          1);  // + 1 because there's one more segment times than segments.

  auto polynomials_start_it = polynomials_.begin() + start_segment_index;
  auto polynomials_slice = vector<PolynomialMatrix>(
      polynomials_start_it, polynomials_start_it + num_segments);

  return PiecewisePolynomial<CoefficientType>(polynomials_slice,
                                              breaks_slice);
}

template <typename CoefficientType>
double PiecewisePolynomial<CoefficientType>::segmentValueAtGlobalAbscissa(
    int segment_index, double t, Eigen::Index row, Eigen::Index col) const {
  return polynomials_[segment_index](row, col).EvaluateUnivariate(
      t - getStartTime(segment_index));
}

template <typename CoefficientType>
Eigen::Index PiecewisePolynomial<CoefficientType>::rows() const {
  if (polynomials_.size() > 0)
    return polynomials_[0].rows();
  else
    throw std::runtime_error(
        "PiecewisePolynomial has no segments. Number of rows is undefined.");
}

template <typename CoefficientType>
Eigen::Index PiecewisePolynomial<CoefficientType>::cols() const {
  if (polynomials_.size() > 0)
    return polynomials_[0].cols();
  else
    throw std::runtime_error(
        "PiecewisePolynomial has no segments. Number of columns is undefined.");
}

// Static generators for splines.

// Throws std::runtime_error if these conditions are true:
//  `breaks` and `knots` have different length,
//  `knots` have inconsistent dimensions,
//  any `knots` have either 0 rows or 0 cols,
//  `breaks` is not strictly increasing,
//  `breaks` has length smaller than `min_length`.
template <typename CoefficientType>
void PiecewisePolynomial<CoefficientType>::
    CheckSplineGenerationInputValidityOrThrow(
        const std::vector<double>& breaks,
        const std::vector<CoefficientMatrix>& knots,
        int min_length) {
  const std::vector<double>& T = breaks;
  const std::vector<CoefficientMatrix>& Y = knots;
  if (T.size() != Y.size()) {
    throw std::runtime_error(
        "Number of break points does not match number of knots.");
  }
  if (static_cast<int>(T.size()) < min_length) {
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
  for (size_t i = 0; i < T.size() - 1; i++) {
    if (T[i + 1] - T[i] < kEpsilonTime) {
      throw std::runtime_error("T must be in increasing order.");
    }
  }
}

// Makes a piecewise constant polynomial.
// The value for each segment is set to the value at the beginning of each
// segment.
template <typename CoefficientType>
PiecewisePolynomial<CoefficientType>
PiecewisePolynomial<CoefficientType>::ZeroOrderHold(
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
            Eigen::Matrix<CoefficientType, 1, 1>(knots[i](j, k)));
      }
    }
    polys.push_back(poly_matrix);
  }
  return PiecewisePolynomial<CoefficientType>(polys, breaks);
}

// Makes a piecewise linear polynomial.
template <typename CoefficientType>
PiecewisePolynomial<CoefficientType>
PiecewisePolynomial<CoefficientType>::FirstOrderHold(
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
        poly_matrix(j, k) = PolynomialType(Eigen::Matrix<CoefficientType, 2, 1>(
            knots[i](j, k), (knots[i + 1](j, k) - knots[i](j, k)) /
                                (breaks[i + 1] - breaks[i])));
      }
    }
    polys.push_back(poly_matrix);
  }
  return PiecewisePolynomial<CoefficientType>(polys, breaks);
}

template <typename CoefficientType>
static int sign(CoefficientType val, CoefficientType tol) {
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
template <typename CoefficientType>
drake::MatrixX<CoefficientType>
PiecewisePolynomial<CoefficientType>::ComputePchipEndSlope(
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
template <typename CoefficientType>
PiecewisePolynomial<CoefficientType>
PiecewisePolynomial<CoefficientType>::Pchip(
    const std::vector<double>& breaks,
    const std::vector<CoefficientMatrix>& knots,
    bool zero_end_point_derivatives) {
  const std::vector<double>& T = breaks;
  const std::vector<CoefficientMatrix>& Y = knots;

  if (zero_end_point_derivatives) {
    CheckSplineGenerationInputValidityOrThrow(T, Y, 2);
  } else {
    CheckSplineGenerationInputValidityOrThrow(T, Y, 3);
  }

  int N = static_cast<int>(T.size());
  int rows = Y.front().rows();
  int cols = Y.front().cols();

  std::vector<PolynomialMatrix> polynomials(N - 1);
  std::vector<CoefficientMatrix> slope(N - 1);
  std::vector<double> dt(N - 1);

  std::vector<CoefficientMatrix> Ydot(N, CoefficientMatrix::Zero(rows, cols));
  Eigen::Matrix<CoefficientType, 4, 1> coeffs;

  // Computes the end slopes.
  CoefficientMatrix Ydot_start = CoefficientMatrix::Zero(rows, cols);
  CoefficientMatrix Ydot_end = CoefficientMatrix::Zero(rows, cols);

  if (!zero_end_point_derivatives) {
    Ydot_start = ComputePchipEndSlope(T[1] - T[0], T[2] - T[1],
                                      (Y[1] - Y[0]) / (T[1] - T[0]),
                                      (Y[2] - Y[1]) / (T[2] - T[1]));
    Ydot_end = ComputePchipEndSlope(
        T[N - 1] - T[N - 2], T[N - 2] - T[N - 3],
        (Y[N - 1] - Y[N - 2]) / (T[N - 1] - T[N - 2]),
        (Y[N - 2] - Y[N - 3]) / (T[N - 2] - T[N - 3]));
  }

  for (int t = 0; t < N - 1; ++t) {
    dt[t] = T[t + 1] - T[t];
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
          CoefficientType common = dt[t] + dt[t + 1];
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

  return PiecewisePolynomial<CoefficientType>(polynomials, T);
}

// Makes a cubic piecewise polynomial using the given knots and their
// derivatives at each break.
template <typename CoefficientType>
PiecewisePolynomial<CoefficientType>
PiecewisePolynomial<CoefficientType>::Cubic(
    const std::vector<double>& breaks,
    const std::vector<CoefficientMatrix>& knots,
    const std::vector<CoefficientMatrix>& knots_dot) {
  const std::vector<double>& T = breaks;
  const std::vector<CoefficientMatrix>& Y = knots;
  const std::vector<CoefficientMatrix>& Ydot = knots_dot;
  CheckSplineGenerationInputValidityOrThrow(T, Y, 2);

  int N = static_cast<int>(T.size());
  int rows = Y.front().rows();
  int cols = Y.front().cols();

  if (T.size() != Ydot.size()) {
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
        double dt = T[t + 1] - T[t];
        Eigen::Matrix<CoefficientType, 4, 1> coeffs = ComputeCubicSplineCoeffs(
            dt, Y[t](i, j), Y[t + 1](i, j), Ydot[t](i, j), Ydot[t + 1](i, j));
        polynomials[t](i, j) = PolynomialType(coeffs);
      }
    }
  }

  return PiecewisePolynomial<CoefficientType>(polynomials, T);
}

// Sets up the linear system for solving for the cubic piecewise polynomial
// coefficients.
// See the header file for more information.
template <typename CoefficientType>
int PiecewisePolynomial<CoefficientType>::
    SetupCubicSplineInteriorCoeffsLinearSystem(
        const std::vector<double>& breaks,
        const std::vector<CoefficientMatrix>& knots,
        int row, int col,
        drake::MatrixX<CoefficientType>* A,
        drake::VectorX<CoefficientType>* b) {
  const std::vector<double>& T = breaks;
  const std::vector<CoefficientMatrix>& Y = knots;
  int N = static_cast<int>(T.size());

  DRAKE_DEMAND(A != nullptr);
  DRAKE_DEMAND(b != nullptr);
  DRAKE_DEMAND(A->rows() == 4 * (N - 1));
  DRAKE_DEMAND(A->cols() == 4 * (N - 1));
  DRAKE_DEMAND(b->rows() == 4 * (N - 1));

  int row_idx = 0;
  drake::MatrixX<CoefficientType>& Aref = *A;
  drake::VectorX<CoefficientType>& bref = *b;

  for (int i = 0; i < N - 1; ++i) {
    double dt = T[i + 1] - T[i];

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
template <typename CoefficientType>
PiecewisePolynomial<CoefficientType>
PiecewisePolynomial<CoefficientType>::Cubic(
    const std::vector<double>& breaks,
    const std::vector<CoefficientMatrix>& knots,
    const CoefficientMatrix& knot_dot_at_start,
    const CoefficientMatrix& knot_dot_at_end) {
  const std::vector<double>& T = breaks;
  const std::vector<CoefficientMatrix>& Y = knots;
  const CoefficientMatrix& Ydot_start = knot_dot_at_start;
  const CoefficientMatrix& Ydot_end = knot_dot_at_end;

  CheckSplineGenerationInputValidityOrThrow(T, Y, 2);

  int N = static_cast<int>(T.size());
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

  drake::MatrixX<CoefficientType> A(4 * (N - 1), 4 * (N - 1));
  drake::VectorX<CoefficientType> b(4 * (N - 1));
  drake::VectorX<CoefficientType> solution;

  A.setZero();
  b.setZero();

  // Sets up a linear equation to solve for the coefficients.
  for (int j = 0; j < rows; ++j) {
    for (int k = 0; k < cols; ++k) {
      int row_idx =
          SetupCubicSplineInteriorCoeffsLinearSystem(T, Y, j, k, &A, &b);

      // Endpoints' velocity matches the given ones.
      A(row_idx, 1) = 1;
      b(row_idx++) = Ydot_start(j, k);

      A(row_idx, 4 * (N - 2) + 1) = 1;
      A(row_idx, 4 * (N - 2) + 2) = 2 * (T[N - 1] - T[N - 2]);
      A(row_idx, 4 * (N - 2) + 3) =
          3 * (T[N - 1] - T[N - 2]) * (T[N - 1] - T[N - 2]);
      b(row_idx++) = Ydot_end(j, k);

      // TODO(siyuan.feng): Should switch to a sparse solver.
      solution = A.colPivHouseholderQr().solve(b);

      for (int i = 0; i < N - 1; ++i) {
        polynomials[i](j, k) =
            Polynomial<CoefficientType>(solution.template segment<4>(4 * i));
      }
    }
  }

  return PiecewisePolynomial<CoefficientType>(polynomials, T);
}

// Makes a cubic piecewise polynomial.
// Internal knot points have continuous values, first and second derivatives,
// Third derivative is also continuous at the end of the first segment and the
// beginning of the last segment.
template <typename CoefficientType>
PiecewisePolynomial<CoefficientType>
PiecewisePolynomial<CoefficientType>::Cubic(
    const std::vector<double>& breaks,
    const std::vector<CoefficientMatrix>& knots) {
  const std::vector<double>& T = breaks;
  const std::vector<CoefficientMatrix>& Y = knots;
  CheckSplineGenerationInputValidityOrThrow(T, Y, 3);

  int N = static_cast<int>(T.size());
  int rows = Y.front().rows();
  int cols = Y.front().cols();

  std::vector<PolynomialMatrix> polynomials(N - 1);
  for (int i = 0; i < N - 1; ++i) {
    polynomials[i].resize(rows, cols);
  }

  drake::MatrixX<CoefficientType> A(4 * (N - 1), 4 * (N - 1));
  drake::VectorX<CoefficientType> b(4 * (N - 1));
  drake::VectorX<CoefficientType> solution;

  A.setZero();
  b.setZero();

  // Sets up a linear equation to solve for the coefficients.
  for (int j = 0; j < rows; ++j) {
    for (int k = 0; k < cols; ++k) {
      int row_idx =
          SetupCubicSplineInteriorCoeffsLinearSystem(T, Y, j, k, &A, &b);

      if (N > 3) {
        // Ydddot(T[1]) is continuous.
        A(row_idx, 3) = 1;
        A(row_idx, 4 + 3) = -1;
        b(row_idx++) = 0;

        // Ydddot(T[N-2]) is continuous.
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

      // TODO(siyuan.feng): Should switch to a sparse solver.
      solution = A.colPivHouseholderQr().solve(b);

      for (int i = 0; i < N - 1; ++i) {
        polynomials[i](j, k) =
            Polynomial<CoefficientType>(solution.template segment<4>(4 * i));
      }
    }
  }

  return PiecewisePolynomial<CoefficientType>(polynomials, T);
}

// Computes the cubic spline coefficients based on the given values and first
// derivatives at both end points.
template <typename CoefficientType>
Eigen::Matrix<CoefficientType, 4, 1>
PiecewisePolynomial<CoefficientType>::ComputeCubicSplineCoeffs(
    double dt, CoefficientType y0, CoefficientType y1, CoefficientType yd0,
    CoefficientType yd1) {
  if (dt < kEpsilonTime) {
    throw std::runtime_error("dt < epsilon.");
  }

  double dt2 = dt * dt;
  CoefficientType c4 = y0;
  CoefficientType c3 = yd0;
  CoefficientType common = (yd1 - c3 - 2. / dt * (y1 - c4 - dt * c3));
  CoefficientType c1 = 1. / dt2 * common;
  CoefficientType c2 = 1. / dt2 * (y1 - c4 - dt * c3 - dt * common);
  return drake::Vector4<CoefficientType>(c4, c3, c2, c1);
}

template class PiecewisePolynomial<double>;
// doesn't work yet
// template class PiecewisePolynomial<std::complex<double>>;
