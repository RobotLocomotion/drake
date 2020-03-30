#include "drake/common/trajectories/piecewise_polynomial.h"

#include <algorithm>
#include <memory>

#include <fmt/format.h>

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
  MatrixX<T> matrix_value_at_start_time =
      MatrixX<T>::Constant(rows(), cols(), value_at_start_time);
  return integral(matrix_value_at_start_time);
}

template <typename T>
PiecewisePolynomial<T> PiecewisePolynomial<T>::integral(
    const Eigen::Ref<MatrixX<T>>& value_at_start_time) const {
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
              matrix(row, col).Integral(ret.EvaluateSegmentAbsoluteTime(
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
  return EvaluateSegmentAbsoluteTime(segment_index, t, row, col);
}

template <typename T>
MatrixX<T> PiecewisePolynomial<T>::EvalDerivative(double t,
                                                  int derivative_order) const {
  const int segment_index = this->get_segment_index(t);
  t = std::min(std::max(t, this->start_time()), this->end_time());
  Eigen::Matrix<double, PolynomialMatrix::RowsAtCompileTime,
                PolynomialMatrix::ColsAtCompileTime>
      ret(rows(), cols());
  for (Eigen::Index row = 0; row < rows(); row++) {
    for (Eigen::Index col = 0; col < cols(); col++) {
      ret(row, col) = EvaluateSegmentAbsoluteTime(segment_index, t, row, col,
                                                   derivative_order);
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
PiecewisePolynomial<T>& PiecewisePolynomial<T>::operator*=(
    const PiecewisePolynomial<T>& other) {
  if (!this->SegmentTimesEqual(other))
    throw runtime_error(
        "Multiplication not yet implemented when segment times are not equal");
  for (size_t i = 0; i < polynomials_.size(); i++) {
    polynomials_[i].array() *= other.polynomials_[i].array();
  }
  return *this;
}

template <typename T>
PiecewisePolynomial<T>& PiecewisePolynomial<T>::operator+=(
    const MatrixX<T>& offset) {
  for (size_t i = 0; i < polynomials_.size(); i++)
    polynomials_[i] += offset.template cast<PolynomialType>();
  return *this;
}

template <typename T>
PiecewisePolynomial<T>& PiecewisePolynomial<T>::operator-=(
    const MatrixX<T>& offset) {
  for (size_t i = 0; i < polynomials_.size(); i++)
    polynomials_[i] -= offset.template cast<PolynomialType>();
  return *this;
}

template <typename T>
const PiecewisePolynomial<T> PiecewisePolynomial<T>::operator+(
    const PiecewisePolynomial<T>& other) const {
  PiecewisePolynomial<T> ret = *this;
  ret += other;
  return ret;
}

template <typename T>
const PiecewisePolynomial<T> PiecewisePolynomial<T>::operator-(
    const PiecewisePolynomial<T>& other) const {
  PiecewisePolynomial<T> ret = *this;
  ret -= other;
  return ret;
}

template <typename T>
const PiecewisePolynomial<T> PiecewisePolynomial<T>::operator*(
    const PiecewisePolynomial<T>& other) const {
  PiecewisePolynomial<T> ret = *this;
  ret *= other;
  return ret;
}

template <typename T>
const PiecewisePolynomial<T> PiecewisePolynomial<T>::operator+(
    const MatrixX<T>& offset) const {
  PiecewisePolynomial<T> ret = *this;
  ret += offset;
  return ret;
}

template <typename T>
const PiecewisePolynomial<T> PiecewisePolynomial<T>::operator-(
    const MatrixX<T>& offset) const {
  PiecewisePolynomial<T> ret = *this;
  ret -= offset;
  return ret;
}

template <typename T>
bool PiecewisePolynomial<T>::isApprox(const PiecewisePolynomial<T>& other,
                                      double tol) const {
  if (rows() != other.rows() || cols() != other.cols()) return false;

  if (!this->SegmentTimesEqual(other, tol)) return false;

  for (int segment_index = 0; segment_index < this->get_number_of_segments();
       segment_index++) {
    const PolynomialMatrix& matrix = polynomials_[segment_index];
    const PolynomialMatrix& other_matrix = other.polynomials_[segment_index];
    for (Eigen::Index row = 0; row < rows(); row++) {
      for (Eigen::Index col = 0; col < cols(); col++) {
        if (!matrix(row, col).IsApprox(other_matrix(row, col), tol)) {
          return false;
        }
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
void PiecewisePolynomial<T>::AppendCubicHermiteSegment(
    double time, const Eigen::Ref<const MatrixX<T>>& sample,
    const Eigen::Ref<const MatrixX<T>>& sample_dot) {
  DRAKE_DEMAND(!empty());
  DRAKE_DEMAND(time > this->end_time());
  DRAKE_DEMAND(sample.rows() == rows());
  DRAKE_DEMAND(sample.cols() == cols());
  DRAKE_DEMAND(sample_dot.rows() == rows());
  DRAKE_DEMAND(sample_dot.cols() == cols());

  const int segment_index = polynomials_.size() - 1;
  const double dt = time - this->end_time();

  PolynomialMatrix matrix(rows(), cols());

  for (int row = 0; row < rows(); ++row) {
    for (int col = 0; col < cols(); ++col) {
      const double start = EvaluateSegmentAbsoluteTime(
          segment_index, this->end_time(), row, col);
      const int derivative_order = 1;
      const double start_dot = EvaluateSegmentAbsoluteTime(
          segment_index, this->end_time(), row, col, derivative_order);
      Vector4<T> coeffs = ComputeCubicSplineCoeffs(
            dt, start, sample(row, col), start_dot, sample_dot(row, col));
      matrix(row, col) = PolynomialType(coeffs);
    }
  }
  polynomials_.push_back(matrix);
  this->get_mutable_breaks().push_back(time);
}

template <typename T>
void PiecewisePolynomial<T>::RemoveFinalSegment() {
  DRAKE_DEMAND(!empty());
  polynomials_.pop_back();
  this->get_mutable_breaks().pop_back();
}

template <typename T>
void PiecewisePolynomial<T>::ReverseTime() {
  using std::pow;
  const std::vector<double>& b = this->breaks();

  // Update the coefficients.
  for (int i = 0; i < this->get_number_of_segments(); i++) {
    PolynomialMatrix& matrix = polynomials_[i];
    const double h = b[i + 1] - b[i];
    for (int row = 0; row < rows(); row++) {
      for (int col = 0; col < cols(); col++) {
        const int d = matrix(row, col).GetDegree();
        if (d == 0) continue;
        const Eigen::VectorXd coeffs = matrix(row, col).GetCoefficients();

        // Must shift this segment by h, because it will now be evaluated
        // relative to breaks[i+1] instead of breaks[i], via p_after(t) =
        // p_before(t+h). This is a slightly involved operation, because
        // substituing (t+h) in a monomial with degree k will effect the
        // coefficients for many monomials.

        // We can perform the time-reversal at the same time, using the variant
        // p_after(t) = p_before(h-t).

        // Compute (h-t) powers, where (h-t)^(j+1) = \sum_j H(j,k) t^(k-1).
        // TODO(russt): For efficiency, I could compute this outside the loop
        // (being careful that every polynomial could have a different
        // degree).
        Eigen::MatrixXd H = Eigen::MatrixXd::Zero(d, d + 1);
        H(0, 0) = h;
        H(0, 1) = -1;
        for (int j = 1; j < d; j++) {
          H.block(j, 0, 1, j + 1) = h * H.block(j - 1, 0, 1, j + 1);
          H.block(j, 1, 1, j + 1) -= H.block(j - 1, 0, 1, j + 1);
        }

        Eigen::VectorXd new_coeffs = Eigen::VectorXd::Zero(d + 1);
        new_coeffs(0) = coeffs(0);
        // Update coefficients.
        for (int j = 0; j < d; j++) {
          new_coeffs += coeffs(j + 1) * H.row(j);
        }

        matrix(row, col) = Polynomial<T>(new_coeffs);
      }
    }
  }

  // Reverse the order of the breaks and polynomials.
  std::vector<double>& breaks = this->get_mutable_breaks();
  std::reverse(breaks.begin(), breaks.end());
  std::reverse(polynomials_.begin(), polynomials_.end());
  // Update the breaks.
  for (auto it = breaks.begin(); it != breaks.end(); ++it) {
    *it *= -1.0;
  }
}

template <typename T>
void PiecewisePolynomial<T>::ScaleTime(double scale) {
  using std::pow;
  DRAKE_DEMAND(scale > 0.0);

  // Update the coefficients.
  for (int i = 0; i < this->get_number_of_segments(); i++) {
    PolynomialMatrix& matrix = polynomials_[i];
    for (int row = 0; row < rows(); row++) {
      for (int col = 0; col < cols(); col++) {
        const int d = matrix(row, col).GetDegree();
        if (d == 0) continue;
        Eigen::VectorXd coeffs = matrix(row, col).GetCoefficients();
        for (int p = 1; p < d + 1; p++) {
          coeffs(p) /= pow(scale, p);
        }
        matrix(row, col) = Polynomial<T>(coeffs);
      }
    }
  }

  // Update the breaks.
  std::vector<double>& breaks = this->get_mutable_breaks();
  for (auto it = breaks.begin(); it != breaks.end(); ++it) {
    *it *= scale;
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
double PiecewisePolynomial<T>::EvaluateSegmentAbsoluteTime(
    int segment_index, double t, Eigen::Index row, Eigen::Index col,
    int derivative_order) const {
  return polynomials_[segment_index](row, col).EvaluateUnivariate(
      t - this->start_time(segment_index), derivative_order);
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
//  `breaks` and `samples` have different length,
//  `samples` have inconsistent dimensions,
//  any `samples` have either 0 rows or 0 cols,
//  `breaks` is not strictly increasing by at least kEpsilonTime per break,
//  `breaks` has length smaller than `min_length`.
template <typename T>
void PiecewisePolynomial<T>::
    CheckSplineGenerationInputValidityOrThrow(
        const std::vector<double>& breaks,
        const std::vector<MatrixX<T>>& samples,
        int min_length) {
  const std::vector<double>& times = breaks;
  const std::vector<MatrixX<T>>& Y = samples;
  if (times.size() != Y.size()) {
    throw std::runtime_error(
        "Number of break points does not match number of samples.");
  }
  if (static_cast<int>(times.size()) < min_length) {
    throw std::runtime_error("Not enough samples.");
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
    if (times[i + 1] <= times[i]) {
      throw std::runtime_error("Times must be in increasing order.");
    }
    if (times[i + 1] - times[i] < PiecewiseTrajectory<T>::kEpsilonTime) {
      throw std::runtime_error(
          fmt::format("Times must be at least {} apart.",
                      PiecewiseTrajectory<T>::kEpsilonTime));
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
    const std::vector<MatrixX<T>>& samples) {
  CheckSplineGenerationInputValidityOrThrow(breaks, samples, 2);

  std::vector<PolynomialMatrix> polys;
  polys.reserve(breaks.size() - 1);
  // For each of the breaks, creates a PolynomialMatrix which can contain joint
  // positions.
  for (int i = 0; i < static_cast<int>(breaks.size()) - 1; ++i) {
    PolynomialMatrix poly_matrix(samples[0].rows(), samples[0].cols());

    for (int j = 0; j < samples[i].rows(); ++j) {
      for (int k = 0; k < samples[i].cols(); ++k) {
        poly_matrix(j, k) = PolynomialType(
            Eigen::Matrix<T, 1, 1>(samples[i](j, k)));
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
    const std::vector<MatrixX<T>>& samples) {
  CheckSplineGenerationInputValidityOrThrow(breaks, samples, 2);

  std::vector<PolynomialMatrix> polys;
  polys.reserve(breaks.size() - 1);
  // For each of the breaks, creates a PolynomialMatrix which can contain joint
  // positions.
  for (int i = 0; i < static_cast<int>(breaks.size()) - 1; ++i) {
    PolynomialMatrix poly_matrix(samples[0].rows(), samples[0].cols());

    for (int j = 0; j < samples[i].rows(); ++j) {
      for (int k = 0; k < samples[i].cols(); ++k) {
        poly_matrix(j, k) = PolynomialType(Eigen::Matrix<T, 2, 1>(
            samples[i](j, k), (samples[i + 1](j, k) - samples[i](j, k)) /
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

namespace {

// Computes the first derivative for either the starting or the end sample
// point.  This is an internal helpful function for pchip.
// The first derivative is computed using a non-centered, shape-preserving
// three-point formulae.
// See equation (2.10) in the following reference for more details.
// http://www.mi.sanu.ac.rs/~gvm/radovi/mon.pdf
template <typename T>
MatrixX<T> ComputePchipEndSlope(double dt0, double dt1,
                                const MatrixX<T>& slope0,
                                const MatrixX<T>& slope1) {
  constexpr T kSlopeEpsilon = 1e-10;
  MatrixX<T> deriv = ((2.0 * dt0 + dt1) * slope0 - dt0 * slope1) / (dt0 + dt1);
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

}  // end namespace

// Makes a cubic piecewise polynomial.
// It first computes the first derivatives at each break, and solves for each
// segment's coefficients using the derivatives and samples.
// The derivatives are computed using a weighted harmonic mean for internal
// points, and ComputePchipEndSlope is used for computing the end points'
// derivatives.
// See pg 9 in http://home.uchicago.edu/~sctchoi/courses/cs138/interp.pdf for
// more details.
template <typename T>
PiecewisePolynomial<T>
PiecewisePolynomial<T>::CubicShapePreserving(
    const std::vector<double>& breaks,
    const std::vector<MatrixX<T>>& samples,
    bool zero_end_point_derivatives) {
  const std::vector<double>& times = breaks;
  const std::vector<MatrixX<T>>& Y = samples;

  if (zero_end_point_derivatives) {
    CheckSplineGenerationInputValidityOrThrow(times, Y, 2);
  } else {
    CheckSplineGenerationInputValidityOrThrow(times, Y, 3);
  }

  int N = static_cast<int>(times.size());
  int rows = Y.front().rows();
  int cols = Y.front().cols();

  std::vector<PolynomialMatrix> polynomials(N - 1);
  std::vector<MatrixX<T>> slope(N - 1);
  std::vector<double> dt(N - 1);

  std::vector<MatrixX<T>> Ydot(N, MatrixX<T>::Zero(rows, cols));
  Eigen::Matrix<T, 4, 1> coeffs;

  // Computes the end slopes.
  MatrixX<T> Ydot_start = MatrixX<T>::Zero(rows, cols);
  MatrixX<T> Ydot_end = MatrixX<T>::Zero(rows, cols);

  if (!zero_end_point_derivatives) {
    Ydot_start =
        ComputePchipEndSlope<T>(times[1] - times[0], times[2] - times[1],
                                (Y[1] - Y[0]) / (times[1] - times[0]),
                                (Y[2] - Y[1]) / (times[2] - times[1]));
    Ydot_end = ComputePchipEndSlope<T>(
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
        // sample[t+1] is local extrema.
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

// Makes a cubic piecewise polynomial using the given samples and their
// derivatives at each break.
template <typename T>
PiecewisePolynomial<T>
PiecewisePolynomial<T>::CubicHermite(
    const std::vector<double>& breaks,
    const std::vector<MatrixX<T>>& samples,
    const std::vector<MatrixX<T>>& samples_dot) {
  const std::vector<double>& times = breaks;
  const std::vector<MatrixX<T>>& Y = samples;
  const std::vector<MatrixX<T>>& Ydot = samples_dot;
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
    const double dt = times[t + 1] - times[t];
    for (int i = 0; i < rows; ++i) {
      for (int j = 0; j < cols; ++j) {
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
        const std::vector<MatrixX<T>>& samples,
        int row, int col,
        MatrixX<T>* A,
        VectorX<T>* b) {
  const std::vector<double>& times = breaks;
  const std::vector<MatrixX<T>>& Y = samples;
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
// Internal sample points have continuous values, first and second derivatives,
// and first derivatives at both end points are set to `sample_dot_at_start`
// and `sample_dot_at_end`.
template <typename T>
PiecewisePolynomial<T>
PiecewisePolynomial<T>::CubicWithContinuousSecondDerivatives(
    const std::vector<double>& breaks,
    const std::vector<MatrixX<T>>& samples,
    const MatrixX<T>& sample_dot_at_start,
    const MatrixX<T>& sample_dot_at_end) {
  const std::vector<double>& times = breaks;
  const std::vector<MatrixX<T>>& Y = samples;
  const MatrixX<T>& Ydot_start = sample_dot_at_start;
  const MatrixX<T>& Ydot_end = sample_dot_at_end;

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
// Internal sample points have continuous values, first and second derivatives.
// If `periodic_end_condition` is `true`, the first and second derivatives will
// be continuous between the end of the last segment and the beginning of
// the first. Otherwise, the third derivative is made continuous between the
// first two segments and between the last two segments (the "not-a-sample"
// end condition).
template <typename T>
PiecewisePolynomial<T>
PiecewisePolynomial<T>::CubicWithContinuousSecondDerivatives(
    const std::vector<double>& breaks,
    const std::vector<MatrixX<T>>& samples,
    bool periodic_end_condition) {
  const std::vector<double>& times = breaks;
  const std::vector<MatrixX<T>>& Y = samples;
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
    const Eigen::Ref<const MatrixX<T>>& samples) {
  DRAKE_DEMAND(samples.cols() == breaks.size());
  std::vector<double> my_breaks(breaks.data(), breaks.data() + breaks.size());
  return PiecewisePolynomial<T>::ZeroOrderHold(my_breaks,
                                               ColsToStdVector(samples));
}

template <typename T>
PiecewisePolynomial<T> PiecewisePolynomial<T>::FirstOrderHold(
    const Eigen::Ref<const Eigen::VectorXd>& breaks,
    const Eigen::Ref<const MatrixX<T>>& samples) {
  DRAKE_DEMAND(samples.cols() == breaks.size());
  std::vector<double> my_breaks(breaks.data(), breaks.data() + breaks.size());
  return PiecewisePolynomial<T>::FirstOrderHold(my_breaks,
                                                ColsToStdVector(samples));
}

template <typename T>
PiecewisePolynomial<T> PiecewisePolynomial<T>::CubicShapePreserving(
    const Eigen::Ref<const Eigen::VectorXd>& breaks,
    const Eigen::Ref<const MatrixX<T>>& samples,
    bool zero_end_point_derivatives) {
  DRAKE_DEMAND(samples.cols() == breaks.size());
  std::vector<double> my_breaks(breaks.data(), breaks.data() + breaks.size());
  return PiecewisePolynomial<T>::CubicShapePreserving(
      my_breaks, ColsToStdVector(samples), zero_end_point_derivatives);
}

template <typename T>
PiecewisePolynomial<T>
PiecewisePolynomial<T>::CubicWithContinuousSecondDerivatives(
    const Eigen::Ref<const Eigen::VectorXd>& breaks,
    const Eigen::Ref<const MatrixX<T>>& samples,
    const Eigen::Ref<const VectorX<T>>& samples_dot_start,
    const Eigen::Ref<const VectorX<T>>& samples_dot_end) {
  DRAKE_DEMAND(samples.cols() == breaks.size());
  std::vector<double> my_breaks(breaks.data(), breaks.data() + breaks.size());
  return PiecewisePolynomial<T>::CubicWithContinuousSecondDerivatives(
      my_breaks, ColsToStdVector(samples), samples_dot_start.eval(),
      samples_dot_end.eval());
}

template <typename T>
PiecewisePolynomial<T> PiecewisePolynomial<T>::CubicHermite(
    const Eigen::Ref<const Eigen::VectorXd>& breaks,
    const Eigen::Ref<const MatrixX<T>>& samples,
    const Eigen::Ref<const MatrixX<T>>& samples_dot) {
  DRAKE_DEMAND(samples.cols() == breaks.size());
  std::vector<double> my_breaks(breaks.data(), breaks.data() + breaks.size());
  return PiecewisePolynomial<T>::CubicHermite(
      my_breaks, ColsToStdVector(samples), ColsToStdVector(samples_dot));
}

template <typename T>
PiecewisePolynomial<T>
PiecewisePolynomial<T>::CubicWithContinuousSecondDerivatives(
    const Eigen::Ref<const Eigen::VectorXd>& breaks,
    const Eigen::Ref<const MatrixX<T>>& samples, bool periodic_end_condition) {
  DRAKE_DEMAND(samples.cols() == breaks.size());
  std::vector<double> my_breaks(breaks.data(), breaks.data() + breaks.size());
  return PiecewisePolynomial<T>::CubicWithContinuousSecondDerivatives(
      my_breaks, ColsToStdVector(samples), periodic_end_condition);
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
