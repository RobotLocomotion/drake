#include "drake/systems/trajectories/PiecewisePolynomial.h"

#include <algorithm>

#include "drake/common/drake_assert.h"
#include "drake/common/test/random_polynomial_matrix.h"

using std::runtime_error;
using std::vector;

template <typename CoefficientType>
PiecewisePolynomial<CoefficientType>::PiecewisePolynomial(
    std::vector<PolynomialMatrix> const& polynomials,
    std::vector<double> const& segment_times)
    : PiecewisePolynomialBase(segment_times), polynomials_(polynomials) {
  DRAKE_ASSERT(segment_times.size() == (polynomials.size() + 1));
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
    std::vector<double> const& segment_times)
    : PiecewisePolynomialBase(segment_times) {
  DRAKE_ASSERT(segment_times.size() == (polynomials.size() + 1));

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
  PiecewisePolynomial ret = *this;
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
              matrix(row, col).Integral(
                  ret.segmentValueAtGlobalAbscissa(
                      segment_index - 1,
                      getStartTime(segment_index), row, col));
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
Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>
PiecewisePolynomial<CoefficientType>::value(double t) const {
  int segment_index = getSegmentIndex(t);
  t = std::min(std::max(t, getStartTime()), getEndTime());
  Eigen::Matrix<double, PolynomialMatrix::RowsAtCompileTime,
                PolynomialMatrix::ColsAtCompileTime> ret(rows(), cols());
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
  if (!segmentTimesEqual(other, 1e-10))
    throw runtime_error(
        "Addition not yet implemented when segment times are not equal");
  for (size_t i = 0; i < polynomials_.size(); i++)
    polynomials_[i] += other.polynomials_[i];
  return *this;
}

template <typename CoefficientType>
PiecewisePolynomial<CoefficientType>& PiecewisePolynomial<CoefficientType>::
operator-=(const PiecewisePolynomial<CoefficientType>& other) {
  if (!segmentTimesEqual(other, 1e-10))
    throw runtime_error(
        "Addition not yet implemented when segment times are not equal");
  for (size_t i = 0; i < polynomials_.size(); i++)
    polynomials_[i] -= other.polynomials_[i];
  return *this;
}

template <typename CoefficientType>
PiecewisePolynomial<CoefficientType>& PiecewisePolynomial<CoefficientType>::
operator*=(const PiecewisePolynomial<CoefficientType>& other) {
  if (!segmentTimesEqual(other, 1e-10))
    throw runtime_error(
        "Multiplication not yet implemented when segment times are not equal");
  for (size_t i = 0; i < polynomials_.size(); i++)
    polynomials_[i] *= other.polynomials_[i];
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
  for (auto it = segment_times.begin(); it != segment_times.end(); ++it) {
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
                                            int num_segments) {
  segmentNumberRangeCheck(start_segment_index);
  segmentNumberRangeCheck(start_segment_index + num_segments - 1);

  auto segment_times_start_it = segment_times.begin() + start_segment_index;
  auto segment_times_slice = vector<double>(
      segment_times_start_it,
      segment_times_start_it + num_segments +
      1);  // + 1 because there's one more segment times than segments

  auto polynomials_start_it = polynomials_.begin() + start_segment_index;
  auto polynomials_slice = vector<PolynomialMatrix>(
      polynomials_start_it, polynomials_start_it + num_segments);

  return PiecewisePolynomial<CoefficientType>(polynomials_slice,
                                              segment_times_slice);
}

template <typename CoefficientType>
double PiecewisePolynomial<CoefficientType>::segmentValueAtGlobalAbscissa(
    int segment_index, double t, Eigen::Index row, Eigen::Index col) const {
  return polynomials_[segment_index](row, col)
      .EvaluateUnivariate(t - getStartTime(segment_index));
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

// TODO(jwnimmer-tri) This method should move into legacy test-only code (in
// other words, some other class and header).  Unseeded randomness leads to
// hard-to-debug failures.
template <typename CoefficientType>
PiecewisePolynomial<CoefficientType> PiecewisePolynomial<
  CoefficientType>::random(Eigen::Index rows, Eigen::Index cols,
                           Eigen::Index num_coefficients_per_polynomial,
                           const std::vector<double>& segment_times) {
  Eigen::Index num_segments =
      static_cast<Eigen::Index>(segment_times.size() - 1);
  std::vector<PolynomialMatrix> polynomials;
  for (Eigen::Index segment_index = 0; segment_index < num_segments;
       ++segment_index) {
    polynomials.push_back(
        drake::test::RandomPolynomialMatrix<CoefficientType>(
            num_coefficients_per_polynomial, rows, cols));
  }
  return PiecewisePolynomial<CoefficientType>(polynomials, segment_times);
}

template class DRAKE_EXPORT PiecewisePolynomial<double>;
// template class DRAKE_EXPORT
// PiecewisePolynomial<std::complex<double>>; // doesn't work yet
