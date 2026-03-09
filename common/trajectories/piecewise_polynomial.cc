#include "drake/common/trajectories/piecewise_polynomial.h"

#include <algorithm>
#include <memory>
#include <utility>

#include <Eigen/SparseCore>
#include <Eigen/SparseLU>
#include <fmt/format.h>

#include "drake/common/drake_assert.h"
#include "drake/common/unused.h"
#include "drake/math/binomial_coefficient.h"
#include "drake/math/matrix_util.h"

using Eigen::VectorXd;
using std::abs;
using std::clamp;
using std::max;
using std::runtime_error;
using std::vector;

namespace drake {
namespace trajectories {

using math::EigenToStdVector;

template <typename T>
PiecewisePolynomial<T>::PiecewisePolynomial(
    const std::vector<PolynomialMatrix>& polynomials,
    const std::vector<T>& breaks)
    : PiecewiseTrajectory<T>(breaks), polynomials_(polynomials) {
  DRAKE_ASSERT(breaks.size() == (polynomials.size() + 1));
  for (int i = 1; i < this->get_number_of_segments(); i++) {
    if (polynomials[i].rows() != polynomials[0].rows()) {
      throw std::runtime_error(
          "The polynomial matrix for each segment must have the same number of "
          "rows.");
    }
    if (polynomials[i].cols() != polynomials[0].cols()) {
      throw std::runtime_error(
          "The polynomial matrix for each segment must have the same number of "
          "columns.");
    }
  }
}

template <typename T>
PiecewisePolynomial<T>::PiecewisePolynomial(
    const std::vector<Polynomial<T>>& polynomials, const std::vector<T>& breaks)
    : PiecewiseTrajectory<T>(breaks) {
  DRAKE_ASSERT(breaks.size() == (polynomials.size() + 1));

  for (size_t i = 0; i < polynomials.size(); i++) {
    PolynomialMatrix matrix(1, 1);
    matrix(0, 0) = polynomials[i];
    polynomials_.push_back(matrix);
  }
}

template <typename T>
PiecewisePolynomial<T>::~PiecewisePolynomial() = default;

template <typename T>
std::unique_ptr<Trajectory<T>> PiecewisePolynomial<T>::DoClone() const {
  return std::make_unique<PiecewisePolynomial<T>>(*this);
}

template <typename T>
std::tuple<std::vector<double>, std::vector<MatrixX<VectorXd>>>
PiecewisePolynomial<T>::GetSerialized() const {
  if constexpr (!std::is_same_v<T, double>) {
    DRAKE_UNREACHABLE();
  } else {
    std::vector<MatrixX<VectorXd>> polynomials(polynomials_.size());
    // Copy the polynomials_'s coefficients into polynomials.
    int max_degree = 0;
    for (int i = 0; i < static_cast<int>(polynomials.size()); ++i) {
      const MatrixX<Polynomial<double>>& ith_in = polynomials_[i];
      MatrixX<VectorXd>& ith_out = polynomials[i];
      ith_out.resize(ith_in.rows(), ith_in.cols());
      for (int j = 0; j < ith_in.rows(); ++j) {
        for (int k = 0; k < ith_in.cols(); ++k) {
          ith_out(j, k) = ith_in(j, k).GetCoefficients();
          max_degree = std::max(max_degree, ith_in(j, k).GetDegree());
        }
      }
    }
    // Always output a square ndarray with shape=(npoly, nrow, ncol, ncoeff).
    for (int i = 0; i < static_cast<int>(polynomials.size()); ++i) {
      MatrixX<VectorXd>& ith_out = polynomials[i];
      for (int j = 0; j < ith_out.rows(); ++j) {
        for (int k = 0; k < ith_out.cols(); ++k) {
          const int old_size = ith_out(j, k).size();
          ith_out(j, k).conservativeResize(max_degree + 1);
          for (int z = old_size; z < max_degree + 1; ++z) {
            ith_out(j, k)(z) = 0.0;
          }
        }
      }
    }
    return std::make_tuple(this->breaks(), std::move(polynomials));
  }
}

template <typename T>
void PiecewisePolynomial<T>::SetSerialized(
    const std::vector<double>& breaks,
    const std::vector<MatrixX<VectorXd>>& polynomials) {
  if constexpr (!std::is_same_v<T, double>) {
    unused(breaks, polynomials);
    DRAKE_UNREACHABLE();
  } else {
    if (breaks.empty() && polynomials.empty()) {
      *this = PiecewisePolynomial<double>();
      return;
    }
    if (breaks.size() != polynomials.size() + 1) {
      throw std::logic_error(fmt::format(
          "PiecewisePolynomial deserialization must provide "
          "len(breaks) == len(polynomials) + 1, but had len(breaks) == {} and "
          "len(polynomials) == {}",
          breaks.size(), polynomials.size()));
    }
    for (int n = 1; n < static_cast<int>(polynomials.size()); ++n) {
      if ((polynomials[n].rows() != polynomials[0].rows()) ||
          (polynomials[n].cols() != polynomials[0].cols())) {
        throw std::logic_error(fmt::format(
            "PiecewisePolynomial deserialization must provide consistently "
            "sized polynomial matrices, but polynomials[{}] had shape ({}, {}) "
            "yet all prior polynomials had shape ({}, {})",
            n, polynomials[n].rows(), polynomials[n].cols(),
            polynomials[0].rows(), polynomials[0].cols()));
      }
    }
    this->get_mutable_breaks() = breaks;
    polynomials_.resize(polynomials.size());
    for (int i = 0; i < static_cast<int>(polynomials.size()); ++i) {
      const MatrixX<VectorXd>& ith_in = polynomials[i];
      MatrixX<Polynomial<double>>& ith_out = polynomials_[i];
      ith_out.resize(ith_in.rows(), ith_in.cols());
      for (int j = 0; j < ith_in.rows(); ++j) {
        for (int k = 0; k < ith_in.cols(); ++k) {
          ith_out(j, k) = Polynomial<double>(ith_in(j, k));
        }
      }
    }
  }
}

template <typename T>
PiecewisePolynomial<T> PiecewisePolynomial<T>::derivative(
    int derivative_order) const {
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
PiecewisePolynomial<T> PiecewisePolynomial<T>::integral(
    const T& value_at_start_time) const {
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
T PiecewisePolynomial<T>::scalarValue(const T& t, Eigen::Index row,
                                      Eigen::Index col) const {
  int segment_index = this->get_segment_index(t);
  return EvaluateSegmentAbsoluteTime(segment_index, t, row, col);
}

template <typename T>
MatrixX<T> PiecewisePolynomial<T>::DoEvalDerivative(
    const T& t, int derivative_order) const {
  const int segment_index = this->get_segment_index(t);
  const T time = clamp(t, this->start_time(), this->end_time());
  Eigen::Matrix<T, PolynomialMatrix::RowsAtCompileTime,
                PolynomialMatrix::ColsAtCompileTime>
      ret(rows(), cols());
  for (Eigen::Index row = 0; row < rows(); row++) {
    for (Eigen::Index col = 0; col < cols(); col++) {
      ret(row, col) = EvaluateSegmentAbsoluteTime(segment_index, time, row, col,
                                                  derivative_order);
    }
  }
  return ret;
}

template <typename T>
const typename PiecewisePolynomial<T>::PolynomialMatrix&
PiecewisePolynomial<T>::getPolynomialMatrix(int segment_index) const {
  return polynomials_[segment_index];
}

template <typename T>
const Polynomial<T>& PiecewisePolynomial<T>::getPolynomial(
    int segment_index, Eigen::Index row, Eigen::Index col) const {
  this->segment_number_range_check(segment_index);
  return polynomials_[segment_index](row, col);
}

template <typename T>
int PiecewisePolynomial<T>::getSegmentPolynomialDegree(int segment_index,
                                                       Eigen::Index row,
                                                       Eigen::Index col) const {
  this->segment_number_range_check(segment_index);
  return polynomials_[segment_index](row, col).GetDegree();
}

template <typename T>
PiecewisePolynomial<T>& PiecewisePolynomial<T>::operator+=(
    const PiecewisePolynomial<T>& other) {
  if (!this->SegmentTimesEqual(other)) {
    throw runtime_error(
        "Addition not yet implemented when segment times are not equal");
  }
  for (size_t i = 0; i < polynomials_.size(); i++)
    polynomials_[i] += other.polynomials_[i];
  return *this;
}

template <typename T>
PiecewisePolynomial<T>& PiecewisePolynomial<T>::operator-=(
    const PiecewisePolynomial<T>& other) {
  if (!this->SegmentTimesEqual(other)) {
    throw runtime_error(
        "Subtraction not yet implemented when segment times are not equal");
  }
  for (size_t i = 0; i < polynomials_.size(); i++)
    polynomials_[i] -= other.polynomials_[i];
  return *this;
}

template <typename T>
PiecewisePolynomial<T>& PiecewisePolynomial<T>::operator*=(
    const PiecewisePolynomial<T>& other) {
  if (!this->SegmentTimesEqual(other)) {
    throw runtime_error(
        "Multiplication not yet implemented when segment times are not equal");
  }
  for (size_t i = 0; i < polynomials_.size(); i++) {
    polynomials_[i] *= other.polynomials_[i];
  }
  return *this;
}

template <typename T>
PiecewisePolynomial<T>& PiecewisePolynomial<T>::operator+=(
    const MatrixX<T>& offset) {
  for (size_t i = 0; i < polynomials_.size(); i++)
    polynomials_[i] += offset.template cast<Polynomial<T>>();
  return *this;
}

template <typename T>
PiecewisePolynomial<T>& PiecewisePolynomial<T>::operator-=(
    const MatrixX<T>& offset) {
  for (size_t i = 0; i < polynomials_.size(); i++)
    polynomials_[i] -= offset.template cast<Polynomial<T>>();
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
const PiecewisePolynomial<T> PiecewisePolynomial<T>::operator-() const {
  PiecewisePolynomial<T> ret = *this;
  for (size_t i = 0; i < polynomials_.size(); i++) {
    ret.polynomials_[i] = -polynomials_[i];
  }
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
                                      double tol,
                                      const ToleranceType& tol_type) const {
  if (rows() != other.rows() || cols() != other.cols()) return false;

  if (!this->SegmentTimesEqual(other, tol)) return false;

  for (int segment_index = 0; segment_index < this->get_number_of_segments();
       segment_index++) {
    const PolynomialMatrix& matrix = polynomials_[segment_index];
    const PolynomialMatrix& other_matrix = other.polynomials_[segment_index];
    for (Eigen::Index row = 0; row < rows(); row++) {
      for (Eigen::Index col = 0; col < cols(); col++) {
        if (!matrix(row, col).CoefficientsAlmostEqual(other_matrix(row, col),
                                                      tol, tol_type)) {
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
    const T time_offset = other.start_time() - this->end_time();
    // Absolute tolerance is scaled along with the time scale.
    const T absolute_tolerance = max(abs(this->end_time()), 1.0) *
                                 std::numeric_limits<double>::epsilon();
    DRAKE_THROW_UNLESS(abs(time_offset) < absolute_tolerance);
    // Gets instance breaks.
    std::vector<T>& breaks = this->get_mutable_breaks();
    // Drops first break to avoid duplication.
    breaks.pop_back();
    // Concatenates other breaks, while shifting them appropriately
    // for both trajectories to be time-aligned.
    for (const T& other_break : other.breaks()) {
      breaks.push_back(other_break - time_offset);
    }
    // Concatenates other polynomials.
    polynomials_.insert(polynomials_.end(), other.polynomials_.begin(),
                        other.polynomials_.end());
  } else {
    std::vector<T>& breaks = this->get_mutable_breaks();
    breaks = other.breaks();
    polynomials_ = other.polynomials_;
  }
}

template <typename T>
void PiecewisePolynomial<T>::AppendCubicHermiteSegment(
    const T& time, const Eigen::Ref<const MatrixX<T>>& sample,
    const Eigen::Ref<const MatrixX<T>>& sample_dot) {
  DRAKE_DEMAND(!empty());
  DRAKE_DEMAND(time > this->end_time());
  DRAKE_DEMAND(sample.rows() == rows());
  DRAKE_DEMAND(sample.cols() == cols());
  DRAKE_DEMAND(sample_dot.rows() == rows());
  DRAKE_DEMAND(sample_dot.cols() == cols());

  const int segment_index = polynomials_.size() - 1;
  const T dt = time - this->end_time();

  PolynomialMatrix matrix(rows(), cols());

  for (int row = 0; row < rows(); ++row) {
    for (int col = 0; col < cols(); ++col) {
      const T start = EvaluateSegmentAbsoluteTime(segment_index,
                                                  this->end_time(), row, col);
      const int derivative_order = 1;
      const T start_dot = EvaluateSegmentAbsoluteTime(
          segment_index, this->end_time(), row, col, derivative_order);
      Vector4<T> coeffs = ComputeCubicSplineCoeffs(
          dt, start, sample(row, col), start_dot, sample_dot(row, col));
      matrix(row, col) = Polynomial<T>(coeffs);
    }
  }
  polynomials_.push_back(std::move(matrix));
  this->get_mutable_breaks().push_back(time);
}

template <typename T>
void PiecewisePolynomial<T>::AppendFirstOrderSegment(
    const T& time, const Eigen::Ref<const MatrixX<T>>& sample) {
  DRAKE_DEMAND(!empty());
  DRAKE_DEMAND(time > this->end_time());
  DRAKE_DEMAND(sample.rows() == rows());
  DRAKE_DEMAND(sample.cols() == cols());

  const int segment_index = polynomials_.size() - 1;
  const T dt = time - this->end_time();

  PolynomialMatrix matrix(rows(), cols());

  for (int row = 0; row < rows(); ++row) {
    for (int col = 0; col < cols(); ++col) {
      const T start = EvaluateSegmentAbsoluteTime(segment_index,
                                                  this->end_time(), row, col);
      matrix(row, col) = Polynomial<T>(
          Eigen::Matrix<T, 2, 1>(start, (sample(row, col) - start) / dt));
    }
  }
  polynomials_.push_back(std::move(matrix));
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
  const std::vector<T>& b = this->breaks();

  // Update the coefficients.
  for (int i = 0; i < this->get_number_of_segments(); i++) {
    PolynomialMatrix& matrix = polynomials_[i];
    const T h = b[i + 1] - b[i];
    for (int row = 0; row < rows(); row++) {
      for (int col = 0; col < cols(); col++) {
        const int d = matrix(row, col).GetDegree();
        if (d == 0) continue;
        // Must shift this segment by h, because it will now be evaluated
        // relative to breaks[i+1] instead of breaks[i], via p_after(t) =
        // p_before(t+h).  But we can perform the time-reversal at the same
        // time, using the variant p_after(t) = p_before(h-t).
        const auto& vars = matrix(row, col).GetVariables();
        DRAKE_ASSERT(vars.size() == 1);
        const typename Polynomial<T>::VarType& t = *vars.begin();
        matrix(row, col) =
            matrix(row, col).Substitute(t, h - Polynomial<T>(1.0, t));
      }
    }
  }

  // Reverse the order of the breaks and polynomials.
  std::vector<T>& breaks = this->get_mutable_breaks();
  std::reverse(breaks.begin(), breaks.end());
  std::reverse(polynomials_.begin(), polynomials_.end());
  // Update the breaks.
  for (auto it = breaks.begin(); it != breaks.end(); ++it) {
    *it *= -1.0;
  }
}

template <typename T>
void PiecewisePolynomial<T>::ScaleTime(const T& scale) {
  using std::pow;
  DRAKE_DEMAND(scale > 0.0);

  // Update the coefficients.
  for (int i = 0; i < this->get_number_of_segments(); i++) {
    PolynomialMatrix& matrix = polynomials_[i];
    for (int row = 0; row < rows(); row++) {
      for (int col = 0; col < cols(); col++) {
        const int d = matrix(row, col).GetDegree();
        if (d == 0) continue;
        VectorX<T> coeffs = matrix(row, col).GetCoefficients();
        for (int p = 1; p < d + 1; p++) {
          coeffs(p) /= pow(scale, p);
        }
        matrix(row, col) = Polynomial<T>(coeffs);
      }
    }
  }

  // Update the breaks.
  std::vector<T>& breaks = this->get_mutable_breaks();
  for (auto it = breaks.begin(); it != breaks.end(); ++it) {
    *it *= scale;
  }
}

template <typename T>
void PiecewisePolynomial<T>::shiftRight(const T& offset) {
  std::vector<T>& breaks = this->get_mutable_breaks();
  for (auto it = breaks.begin(); it != breaks.end(); ++it) {
    *it += offset;
  }
}

namespace {
template <typename T>
Polynomial<T> ShiftPoly(Polynomial<T> poly, const T& x) {
  if (poly.GetVariables().size() == 0) {
    // Make constant polynomial.
    return poly;
  }
  using std::pow;
  // Given p(t) = a0 + a1*t + a2*t^2 + ... + an*t^n,
  // we want to shift the parameter to p(t + x)
  // = b0 + b1*t + b2*t^2 + ... + bn*t^n.
  // We can expand the rhs to get the values in b.
  DRAKE_THROW_UNLESS(poly.GetVariables().size() == 1);
  int n = poly.GetDegree();
  const auto& a = poly.GetCoefficients();
  Eigen::Matrix<T, Eigen::Dynamic, 1> b =
      Eigen::Matrix<T, Eigen::Dynamic, 1>::Zero(n + 1);
  for (int i = 0; i <= n; i++) {
    for (int j = 0; j <= i; j++) {
      b(j) += a(i) * drake::math::BinomialCoefficient(i, j) * pow(x, i - j);
    }
  }
  return Polynomial<T>(b);
}
}  // namespace

template <typename T>
int PiecewisePolynomial<T>::AddBreak(const T& new_break) {
  auto& breaks = this->get_mutable_breaks();
  // Check if the new break is already within the kEpsilonTime of an
  // existing break.
  for (int k = 0; k < this->get_number_of_segments(); k++) {
    const T& break_time = breaks[k];
    if (abs(break_time - new_break) < PiecewiseTrajectory<T>::kEpsilonTime) {
      // No need to add a new break.
      return k;
    }
  }
  DRAKE_THROW_UNLESS(this->is_time_in_range(new_break));
  const auto value = this->value(new_break);
  int i = this->get_segment_index(new_break);
  const T last_break_before_new = this->start_time(i);
  const T shift = new_break - last_break_before_new;
  breaks.insert(breaks.begin() + i + 1, new_break);
  // New polynomial matrix to be added at index i.
  const auto& broken_polynomial = polynomials_[i];
  PolynomialMatrix new_polynomial = broken_polynomial;
  for (int row = 0; row < rows(); row++) {
    for (int col = 0; col < cols(); col++) {
      new_polynomial(row, col) = ShiftPoly(new_polynomial(row, col), shift);
    }
  }
  // The i+1'th polynomial should become the new_polynomial.
  polynomials_.insert(polynomials_.begin() + i + 1, new_polynomial);
  return i + 1;
}

template <typename T>
PiecewisePolynomial<T> PiecewisePolynomial<T>::SliceByTime(
    const T& start_time, const T& end_time) const {
  DRAKE_THROW_UNLESS(start_time < end_time);
  DRAKE_THROW_UNLESS(this->is_time_in_range(start_time));
  DRAKE_THROW_UNLESS(this->is_time_in_range(end_time));
  PiecewisePolynomial<T> result{*this};
  int i = result.AddBreak(start_time);
  int j = result.AddBreak(end_time);
  std::vector<PolynomialMatrix> polynomials;
  std::vector<T> breaks;
  for (int k = i; k < j; k++) {
    polynomials.push_back(result.getPolynomialMatrix(k));
    breaks.push_back(result.get_segment_times()[k]);
  }
  // Add the last break
  breaks.push_back(result.get_segment_times()[j]);
  return PiecewisePolynomial<T>(polynomials, breaks);
}

template <typename T>
void PiecewisePolynomial<T>::setPolynomialMatrixBlock(
    const typename PiecewisePolynomial<T>::PolynomialMatrix& replacement,
    int segment_number, Eigen::Index row_start, Eigen::Index col_start) {
  this->segment_number_range_check(segment_number);
  polynomials_[segment_number].block(row_start, col_start, replacement.rows(),
                                     replacement.cols()) = replacement;
}

template <typename T>
PiecewisePolynomial<T> PiecewisePolynomial<T>::slice(int start_segment_index,
                                                     int num_segments) const {
  this->segment_number_range_check(start_segment_index);
  this->segment_number_range_check(start_segment_index + num_segments - 1);

  auto breaks_start_it = this->breaks().begin() + start_segment_index;
  auto breaks_slice = vector<T>(
      breaks_start_it,
      breaks_start_it + num_segments +
          1);  // + 1 because there's one more segment times than segments.

  auto polynomials_start_it = polynomials_.begin() + start_segment_index;
  auto polynomials_slice = vector<PolynomialMatrix>(
      polynomials_start_it, polynomials_start_it + num_segments);

  return PiecewisePolynomial<T>(polynomials_slice, breaks_slice);
}

template <typename T>
T PiecewisePolynomial<T>::EvaluateSegmentAbsoluteTime(
    int segment_index, const T& t, Eigen::Index row, Eigen::Index col,
    int derivative_order) const {
  DRAKE_DEMAND(static_cast<int>(polynomials_.size()) > segment_index);
  return polynomials_[segment_index](row, col).EvaluateUnivariate(
      t - this->start_time(segment_index), derivative_order);
}

template <typename T>
Eigen::Index PiecewisePolynomial<T>::do_rows() const {
  if (polynomials_.size() > 0) {
    return polynomials_[0].rows();
  } else {
    throw std::runtime_error(
        "PiecewisePolynomial has no segments. Number of rows is undefined.");
  }
}

template <typename T>
Eigen::Index PiecewisePolynomial<T>::do_cols() const {
  if (polynomials_.size() > 0) {
    return polynomials_[0].cols();
  } else {
    throw std::runtime_error(
        "PiecewisePolynomial has no segments. Number of columns is undefined.");
  }
}

template <typename T>
void PiecewisePolynomial<T>::Reshape(int rows, int cols) {
  DRAKE_DEMAND(rows * cols == this->rows() * this->cols());
  for (auto& p : polynomials_) {
    // Accordining to the Eigen documentation, data is preserved when the total
    // number of elements does not change.
    p.resize(rows, cols);
  }
}

template <typename T>
PiecewisePolynomial<T> PiecewisePolynomial<T>::Transpose() const {
  std::vector<PolynomialMatrix> transposed;
  std::transform(polynomials_.begin(), polynomials_.end(),
                 std::back_inserter(transposed),
                 [](const PolynomialMatrix& matrix) {
                   return matrix.transpose();
                 });
  return PiecewisePolynomial<T>(transposed, this->breaks());
}

template <typename T>
PiecewisePolynomial<T> PiecewisePolynomial<T>::Block(int start_row,
                                                     int start_col,
                                                     int block_rows,
                                                     int block_cols) const {
  DRAKE_DEMAND(start_row >= 0 && start_row < rows());
  DRAKE_DEMAND(start_col >= 0 && start_col < cols());
  DRAKE_DEMAND(block_rows >= 0 && start_row + block_rows <= rows());
  DRAKE_DEMAND(block_cols >= 0 && start_col + block_cols <= cols());

  std::vector<PolynomialMatrix> block_polynomials;
  std::transform(polynomials_.begin(), polynomials_.end(),
                 std::back_inserter(block_polynomials),
                 [start_row, start_col, block_rows,
                  block_cols](const PolynomialMatrix& matrix) {
                   return matrix.block(start_row, start_col, block_rows,
                                       block_cols);
                 });
  return PiecewisePolynomial<T>(block_polynomials, this->breaks());
}

// Static generators for splines.

// Throws std::runtime_error if these conditions are true:
//  `breaks` and `samples` have different length,
//  `samples` have inconsistent dimensions,
//  any `samples` have either 0 rows or 0 cols,
//  `breaks` is not strictly increasing by at least kEpsilonTime per break,
//  `breaks` has length smaller than `min_length`.
template <typename T>
void PiecewisePolynomial<T>::CheckSplineGenerationInputValidityOrThrow(
    const std::vector<T>& breaks, const std::vector<MatrixX<T>>& samples,
    int min_length) {
  const std::vector<T>& times = breaks;
  const std::vector<MatrixX<T>>& Y = samples;
  if (times.size() != Y.size()) {
    throw std::runtime_error(fmt::format(
        "Number of break points {} does not match number of samples {}.",
        times.size(), Y.size()));
  }
  if (static_cast<int>(times.size()) < min_length) {
    throw std::runtime_error(fmt::format(
        "{} samples is not enough samples (this method requires at least {}).",
        times.size(), min_length));
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
PiecewisePolynomial<T> PiecewisePolynomial<T>::ZeroOrderHold(
    const std::vector<T>& breaks, const std::vector<MatrixX<T>>& samples) {
  CheckSplineGenerationInputValidityOrThrow(breaks, samples, 2);

  std::vector<PolynomialMatrix> polys;
  polys.reserve(breaks.size() - 1);
  // For each of the breaks, creates a PolynomialMatrix which can contain joint
  // positions.
  for (int i = 0; i < static_cast<int>(breaks.size()) - 1; ++i) {
    PolynomialMatrix poly_matrix(samples[0].rows(), samples[0].cols());

    for (int j = 0; j < samples[i].rows(); ++j) {
      for (int k = 0; k < samples[i].cols(); ++k) {
        poly_matrix(j, k) =
            Polynomial<T>(Eigen::Matrix<T, 1, 1>(samples[i](j, k)));
      }
    }
    polys.push_back(poly_matrix);
  }
  return PiecewisePolynomial<T>(polys, breaks);
}

// Makes a piecewise linear polynomial.
template <typename T>
PiecewisePolynomial<T> PiecewisePolynomial<T>::FirstOrderHold(
    const std::vector<T>& breaks, const std::vector<MatrixX<T>>& samples) {
  CheckSplineGenerationInputValidityOrThrow(breaks, samples, 2);

  std::vector<PolynomialMatrix> polys;
  polys.reserve(breaks.size() - 1);
  // For each of the breaks, creates a PolynomialMatrix which can contain joint
  // positions.
  for (int i = 0; i < static_cast<int>(breaks.size()) - 1; ++i) {
    PolynomialMatrix poly_matrix(samples[0].rows(), samples[0].cols());

    for (int j = 0; j < samples[i].rows(); ++j) {
      for (int k = 0; k < samples[i].cols(); ++k) {
        poly_matrix(j, k) = Polynomial<T>(Eigen::Matrix<T, 2, 1>(
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
  if (val < -tol) {
    return -1;
  } else if (val > tol) {
    return 1;
  }
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
MatrixX<T> ComputePchipEndSlope(const T& dt0, const T& dt1,
                                const MatrixX<T>& slope0,
                                const MatrixX<T>& slope1) {
  const T kSlopeEpsilon = 1e-10;
  MatrixX<T> deriv = ((2.0 * dt0 + dt1) * slope0 - dt0 * slope1) / (dt0 + dt1);
  for (int i = 0; i < deriv.rows(); ++i) {
    for (int j = 0; j < deriv.cols(); ++j) {
      if (sign(deriv(i, j), kSlopeEpsilon) !=
          sign(slope0(i, j), kSlopeEpsilon)) {
        deriv(i, j) = 0.;
      } else if (sign(slope0(i, j), kSlopeEpsilon) !=
                     sign(slope1(i, j), kSlopeEpsilon) &&
                 abs(deriv(i, j)) > abs(3. * slope0(i, j))) {
        deriv(i, j) = 3. * slope0(i, j);
      }
    }
  }
  return deriv;
}

}  // namespace

// Makes a cubic piecewise polynomial.
// It first computes the first derivatives at each break, and solves for each
// segment's coefficients using the derivatives and samples.
// The derivatives are computed using a weighted harmonic mean for internal
// points, and ComputePchipEndSlope is used for computing the end points'
// derivatives.
// See pg 9 in http://home.uchicago.edu/~sctchoi/courses/cs138/interp.pdf for
// more details.
template <typename T>
PiecewisePolynomial<T> PiecewisePolynomial<T>::CubicShapePreserving(
    const std::vector<T>& breaks, const std::vector<MatrixX<T>>& samples,
    bool zero_end_point_derivatives) {
  const std::vector<T>& times = breaks;
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
  std::vector<T> dt(N - 1);

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
          Ydot[t + 1](j, k) = 3 * common /
                              ((common + dt[t + 1]) / slope[t](j, k) +
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
        polynomials[t](j, k) = Polynomial<T>(coeffs);
      }
    }
  }

  return PiecewisePolynomial<T>(polynomials, times);
}

// Makes a cubic piecewise polynomial using the given samples and their
// derivatives at each break.
template <typename T>
PiecewisePolynomial<T> PiecewisePolynomial<T>::CubicHermite(
    const std::vector<T>& breaks, const std::vector<MatrixX<T>>& samples,
    const std::vector<MatrixX<T>>& samples_dot) {
  const std::vector<T>& times = breaks;
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
    const T dt = times[t + 1] - times[t];
    for (int i = 0; i < rows; ++i) {
      for (int j = 0; j < cols; ++j) {
        Eigen::Matrix<T, 4, 1> coeffs = ComputeCubicSplineCoeffs(
            dt, Y[t](i, j), Y[t + 1](i, j), Ydot[t](i, j), Ydot[t + 1](i, j));
        polynomials[t](i, j) = Polynomial<T>(coeffs);
      }
    }
  }

  return PiecewisePolynomial<T>(polynomials, times);
}

// Sets up the linear system for solving for the cubic piecewise polynomial
// coefficients.
// See the header file for more information.
template <typename T>
int PiecewisePolynomial<T>::SetupCubicSplineInteriorCoeffsLinearSystem(
    const std::vector<T>& breaks, const std::vector<MatrixX<T>>& samples,
    int row, int col, std::vector<Eigen::Triplet<T>>* triplet_list,
    VectorX<T>* b) {
  const std::vector<T>& times = breaks;
  const std::vector<MatrixX<T>>& Y = samples;
  int N = static_cast<int>(times.size());

  DRAKE_DEMAND(triplet_list != nullptr);
  DRAKE_DEMAND(b != nullptr);
  DRAKE_DEMAND(b->rows() == 3 * (N - 1));

  int row_idx = 0;
  std::vector<Eigen::Triplet<T>>& triplet_ref = *triplet_list;
  VectorX<T>& bref = *b;

  for (int i = 0; i < N - 1; ++i) {
    const T dt = times[i + 1] - times[i];

    // y_i(x_{i+1}) = y_{i+1}(x_{i}) =>
    // Y[i] + a1i*(x_{i+1} - x_i) + a2i(x_{i+1} - x_i)^2 + a3i(x_{i+1} -
    // x_i)^3 = Y[i+1]
    triplet_ref.push_back(Eigen::Triplet<T>(row_idx, 3 * i + 0, dt));
    triplet_ref.push_back(Eigen::Triplet<T>(row_idx, 3 * i + 1, dt * dt));
    triplet_ref.push_back(Eigen::Triplet<T>(row_idx, 3 * i + 2, dt * dt * dt));
    bref(row_idx++) = Y[i + 1](row, col) - Y[i](row, col);

    // y_i'(x_{i+1}) = y_{i+1}'(x_{i}) =>
    // a1i + 2*a2i(x_{i+1} - x_i) + 3*a3i(x_{i+1} - x_i)^2 = a1{i+1}
    if (i != N - 2) {
      triplet_ref.push_back(Eigen::Triplet<T>(row_idx, 3 * i + 0, 1));
      triplet_ref.push_back(Eigen::Triplet<T>(row_idx, 3 * i + 1, 2 * dt));
      triplet_ref.push_back(Eigen::Triplet<T>(row_idx, 3 * i + 2, 3 * dt * dt));
      triplet_ref.push_back(Eigen::Triplet<T>(row_idx++, 3 * (i + 1), -1));
    }

    if (i != N - 2) {
      // y_i''(x_{i+1}) = y_{i+1}''(x_{i}) =>
      // 2*a2i + 6*a3i(x_{i+1} - x_i) = 2*a2{i+1}
      triplet_ref.push_back(Eigen::Triplet<T>(row_idx, 3 * i + 1, 2));
      triplet_ref.push_back(Eigen::Triplet<T>(row_idx, 3 * i + 2, 6 * dt));
      triplet_ref.push_back(Eigen::Triplet<T>(row_idx++, 3 * (i + 1) + 1, -2));
    }
  }
  DRAKE_DEMAND(row_idx == 3 * (N - 1) - 2);
  return row_idx;
}

// Makes a cubic piecewise polynomial.
// Internal sample points have continuous values, first and second derivatives,
// and first derivatives at both end points are set to `sample_dot_at_start`
// and `sample_dot_at_end`.
template <typename T>
PiecewisePolynomial<T>
PiecewisePolynomial<T>::CubicWithContinuousSecondDerivatives(
    const std::vector<T>& breaks, const std::vector<MatrixX<T>>& samples,
    const MatrixX<T>& sample_dot_at_start,
    const MatrixX<T>& sample_dot_at_end) {
  const std::vector<T>& times = breaks;
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

  Eigen::SparseMatrix<T> A(3 * (N - 1), 3 * (N - 1));
  VectorX<T> b(3 * (N - 1));
  VectorX<T> solution;
  VectorX<T> coeffs(4);
  Eigen::SparseLU<Eigen::SparseMatrix<T>> solver;

  b.setZero();

  // Sets up a linear equation to solve for the coefficients.
  for (int j = 0; j < rows; ++j) {
    for (int k = 0; k < cols; ++k) {
      std::vector<Eigen::Triplet<T>> triplet_list;
      // 10 coefficients are needed for the constraints that ensure continuity
      // between adjacent segments: 3 for position, 4 for velocity, 3 for
      // acceleration.  The additional coefficients consist of: 3 for final
      // position constraint, 1 for initial velocity constraint & 3 for final
      // velocity constraint.
      triplet_list.reserve(10 * (N - 2) + 7);
      int row_idx = SetupCubicSplineInteriorCoeffsLinearSystem(
          times, Y, j, k, &triplet_list, &b);

      // Endpoints' velocity matches the given ones.
      triplet_list.push_back(Eigen::Triplet<T>(row_idx, 0, 1));
      b(row_idx++) = Ydot_start(j, k);

      triplet_list.push_back(Eigen::Triplet<T>(row_idx, 3 * (N - 2) + 0, 1));
      triplet_list.push_back(Eigen::Triplet<T>(
          row_idx, 3 * (N - 2) + 1, 2 * (times[N - 1] - times[N - 2])));
      triplet_list.push_back(Eigen::Triplet<T>(
          row_idx, 3 * (N - 2) + 2,
          3 * (times[N - 1] - times[N - 2]) * (times[N - 1] - times[N - 2])));
      b(row_idx++) = Ydot_end(j, k);

      A.setFromTriplets(triplet_list.begin(), triplet_list.end());
      if (j == 0 && k == 0) {
        solver.analyzePattern(A);
      }
      solver.factorize(A);
      solution = solver.solve(b);

      for (int i = 0; i < N - 1; ++i) {
        coeffs(0) = Y[i](j, k);
        coeffs.tail(3) = solution.template segment<3>(3 * i);
        polynomials[i](j, k) = Polynomial<T>(coeffs);
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
    const std::vector<T>& breaks, const std::vector<MatrixX<T>>& samples,
    bool periodic_end_condition) {
  const std::vector<T>& times = breaks;
  const std::vector<MatrixX<T>>& Y = samples;
  CheckSplineGenerationInputValidityOrThrow(times, Y, 3);

  int N = static_cast<int>(times.size());
  int rows = Y.front().rows();
  int cols = Y.front().cols();

  std::vector<PolynomialMatrix> polynomials(N - 1);
  for (int i = 0; i < N - 1; ++i) {
    polynomials[i].resize(rows, cols);
  }

  Eigen::SparseMatrix<T> A(3 * (N - 1), 3 * (N - 1));
  VectorX<T> b(3 * (N - 1));
  VectorX<T> solution;
  VectorX<T> coeffs(4);
  Eigen::SparseLU<Eigen::SparseMatrix<T>> solver;

  b.setZero();

  // Sets up a linear equation to solve for the coefficients.
  for (int j = 0; j < rows; ++j) {
    for (int k = 0; k < cols; ++k) {
      std::vector<Eigen::Triplet<T>> triplet_list;
      // 10 coefficients are needed for the constraints that ensure continuity
      // between adjacent segments: 3 for position, 4 for velocity, 3 for
      // acceleration.  The additional coefficients consist of: 3 for final
      // position constraint, 4 for periodic velocity constraint & 3 for
      // periodic acceleration constraint.  In the case of "not-a-sample" end
      // condition, fewer coefficients are needed.
      triplet_list.reserve(10 * (N - 2) + 10);
      int row_idx = SetupCubicSplineInteriorCoeffsLinearSystem(
          times, Y, j, k, &triplet_list, &b);

      if (periodic_end_condition) {
        // Time during the last segment.
        const T end_dt = times[times.size() - 1] - times[times.size() - 2];
        // Enforce velocity between end-of-last and beginning-of-first segments
        // is continuous.
        // Linear term of 1st segment.
        triplet_list.push_back(Eigen::Triplet<T>(row_idx, 0, -1));
        // Linear term of last segment.
        triplet_list.push_back(Eigen::Triplet<T>(row_idx, 3 * (N - 2) + 0, 1));
        // Squared term of last segment.
        triplet_list.push_back(
            Eigen::Triplet<T>(row_idx, 3 * (N - 2) + 1, 2 * end_dt));
        // Cubic term of last segment.
        triplet_list.push_back(
            Eigen::Triplet<T>(row_idx, 3 * (N - 2) + 2, 3 * end_dt * end_dt));
        b(row_idx++) = 0;

        // Enforce that acceleration between end-of-last and beginning-of-first
        // segments is continuous.
        // Quadratic term of 1st segment.
        triplet_list.push_back(Eigen::Triplet<T>(row_idx, 1, -2));
        // Quadratic term of last segment.
        triplet_list.push_back(Eigen::Triplet<T>(row_idx, 3 * (N - 2) + 1, 2));
        // Cubic term of last segment.
        triplet_list.push_back(
            Eigen::Triplet<T>(row_idx, 3 * (N - 2) + 2, 6 * end_dt));
        b(row_idx++) = 0;
      } else {
        if (N > 3) {
          // Ydddot(times[1]) is continuous.
          // Cubic term of 1st segment.
          triplet_list.push_back(Eigen::Triplet<T>(row_idx, 2, 1));
          // Cubic term of 2nd segment.
          triplet_list.push_back(Eigen::Triplet<T>(row_idx, 3 + 2, -1));
          b(row_idx++) = 0;

          // Ydddot(times[N-2]) is continuous.
          triplet_list.push_back(
              Eigen::Triplet<T>(row_idx, 3 * (N - 3) + 2, 1));
          triplet_list.push_back(
              Eigen::Triplet<T>(row_idx, 3 * (N - 2) + 2, -1));
          b(row_idx++) = 0;
        } else {
          // Set Jerk to zero if only have 3 points, becomes a quadratic.
          triplet_list.push_back(Eigen::Triplet<T>(row_idx, 2, 1));
          b(row_idx++) = 0;

          triplet_list.push_back(Eigen::Triplet<T>(row_idx, 3 + 2, 1));
          b(row_idx++) = 0;
        }
      }

      A.setFromTriplets(triplet_list.begin(), triplet_list.end());
      if (j == 0 && k == 0) {
        solver.analyzePattern(A);
      }
      solver.factorize(A);
      solution = solver.solve(b);

      for (int i = 0; i < N - 1; ++i) {
        coeffs(0) = Y[i](j, k);
        coeffs.tail(3) = solution.template segment<3>(3 * i);
        polynomials[i](j, k) = Polynomial<T>(coeffs);
      }
    }
  }

  return PiecewisePolynomial<T>(polynomials, times);
}

template <typename T>
PiecewisePolynomial<T> PiecewisePolynomial<T>::LagrangeInterpolatingPolynomial(
    const std::vector<T>& times, const std::vector<MatrixX<T>>& samples) {
  using std::pow;

  // Check the inputs.
  DRAKE_DEMAND(times.size() > 1);
  DRAKE_DEMAND(samples.size() == times.size());
  const int rows = samples[0].rows();
  const int cols = samples[0].cols();
  for (size_t i = 1; i < times.size(); ++i) {
    DRAKE_DEMAND(times[i] - times[i - 1] >
                 PiecewiseTrajectory<T>::kEpsilonTime);
    DRAKE_DEMAND(samples[i].rows() == rows);
    DRAKE_DEMAND(samples[i].cols() == cols);
  }

  // https://en.wikipedia.org/wiki/Polynomial_interpolation notes that
  // this Vandermonde matrix can be poorly conditioned if times are close
  // together, and suggests that there are special O(n^2) algorithms available
  // for this particular problem.  But we just implement the simple algorithm
  // here for now.

  // Set up the system of linear equations to solve for the coefficients.
  MatrixX<T> A(times.size(), times.size());
  VectorX<T> b(times.size());

  // Only need to set up the A matrix once.
  for (size_t i = 0; i < times.size(); ++i) {
    const T relative_time = times[i] - times[0];
    A(i, 0) = 1.0;
    for (size_t j = 1; j < times.size(); ++j) {
      A(i, j) = A(i, j - 1) * relative_time;
    }
  }
  Eigen::ColPivHouseholderQR<MatrixX<T>> Aqr(A);

  // Solve for the coefficient matrices.
  PolynomialMatrix polynomials(rows, cols);
  for (int i = 0; i < rows; ++i) {
    for (int j = 0; j < cols; ++j) {
      for (size_t k = 0; k < times.size(); ++k) {
        b(k) = samples[k](i, j);
      }
      polynomials(i, j) = Polynomial<T>(Aqr.solve(b));
    }
  }

  return PiecewisePolynomial<T>({polynomials},
                                {times[0], times[times.size() - 1]});
}

template <typename T>
PiecewisePolynomial<T> PiecewisePolynomial<T>::ZeroOrderHold(
    const Eigen::Ref<const VectorX<T>>& breaks,
    const Eigen::Ref<const MatrixX<T>>& samples) {
  DRAKE_DEMAND(samples.cols() == breaks.size());
  std::vector<T> my_breaks(breaks.data(), breaks.data() + breaks.size());
  return PiecewisePolynomial<T>::ZeroOrderHold(my_breaks,
                                               EigenToStdVector(samples));
}

template <typename T>
PiecewisePolynomial<T> PiecewisePolynomial<T>::FirstOrderHold(
    const Eigen::Ref<const VectorX<T>>& breaks,
    const Eigen::Ref<const MatrixX<T>>& samples) {
  DRAKE_DEMAND(samples.cols() == breaks.size());
  std::vector<T> my_breaks(breaks.data(), breaks.data() + breaks.size());
  return PiecewisePolynomial<T>::FirstOrderHold(my_breaks,
                                                EigenToStdVector(samples));
}

template <typename T>
PiecewisePolynomial<T> PiecewisePolynomial<T>::CubicShapePreserving(
    const Eigen::Ref<const VectorX<T>>& breaks,
    const Eigen::Ref<const MatrixX<T>>& samples,
    bool zero_end_point_derivatives) {
  DRAKE_DEMAND(samples.cols() == breaks.size());
  std::vector<T> my_breaks(breaks.data(), breaks.data() + breaks.size());
  return PiecewisePolynomial<T>::CubicShapePreserving(
      my_breaks, EigenToStdVector(samples), zero_end_point_derivatives);
}

template <typename T>
PiecewisePolynomial<T>
PiecewisePolynomial<T>::CubicWithContinuousSecondDerivatives(
    const Eigen::Ref<const VectorX<T>>& breaks,
    const Eigen::Ref<const MatrixX<T>>& samples,
    const Eigen::Ref<const VectorX<T>>& samples_dot_start,
    const Eigen::Ref<const VectorX<T>>& samples_dot_end) {
  DRAKE_DEMAND(samples.cols() == breaks.size());
  std::vector<T> my_breaks(breaks.data(), breaks.data() + breaks.size());
  return PiecewisePolynomial<T>::CubicWithContinuousSecondDerivatives(
      my_breaks, EigenToStdVector(samples), samples_dot_start.eval(),
      samples_dot_end.eval());
}

template <typename T>
PiecewisePolynomial<T> PiecewisePolynomial<T>::CubicHermite(
    const Eigen::Ref<const VectorX<T>>& breaks,
    const Eigen::Ref<const MatrixX<T>>& samples,
    const Eigen::Ref<const MatrixX<T>>& samples_dot) {
  DRAKE_DEMAND(samples.cols() == breaks.size());
  std::vector<T> my_breaks(breaks.data(), breaks.data() + breaks.size());
  return PiecewisePolynomial<T>::CubicHermite(
      my_breaks, EigenToStdVector(samples), EigenToStdVector(samples_dot));
}

template <typename T>
PiecewisePolynomial<T>
PiecewisePolynomial<T>::CubicWithContinuousSecondDerivatives(
    const Eigen::Ref<const VectorX<T>>& breaks,
    const Eigen::Ref<const MatrixX<T>>& samples, bool periodic_end_condition) {
  DRAKE_DEMAND(samples.cols() == breaks.size());
  std::vector<T> my_breaks(breaks.data(), breaks.data() + breaks.size());
  return PiecewisePolynomial<T>::CubicWithContinuousSecondDerivatives(
      my_breaks, EigenToStdVector(samples), periodic_end_condition);
}

template <typename T>
PiecewisePolynomial<T> PiecewisePolynomial<T>::LagrangeInterpolatingPolynomial(
    const Eigen::Ref<const VectorX<T>>& times,
    const Eigen::Ref<const MatrixX<T>>& samples) {
  DRAKE_DEMAND(samples.cols() == times.size());
  std::vector<T> my_times(times.data(), times.data() + times.size());
  return PiecewisePolynomial<T>::LagrangeInterpolatingPolynomial(
      my_times, EigenToStdVector(samples));
}

// Computes the cubic spline coefficients based on the given values and first
// derivatives at both end points.
template <typename T>
Eigen::Matrix<T, 4, 1> PiecewisePolynomial<T>::ComputeCubicSplineCoeffs(
    const T& dt, T y0, T y1, T yd0, T yd1) {
  if (dt < PiecewiseTrajectory<T>::kEpsilonTime) {
    throw std::runtime_error("dt < epsilon.");
  }

  T dt2 = dt * dt;
  T c4 = y0;
  T c3 = yd0;
  T common = (yd1 - c3 - 2. / dt * (y1 - c4 - dt * c3));
  T c1 = 1. / dt2 * common;
  T c2 = 1. / dt2 * (y1 - c4 - dt * c3 - dt * common);
  return Vector4<T>(c4, c3, c2, c1);
}

}  // namespace trajectories
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class drake::trajectories::PiecewisePolynomial);
