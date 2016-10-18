#include "drake/systems/trajectories/ExponentialPlusPiecewisePolynomial.h"
#include <unsupported/Eigen/MatrixFunctions>
#include <stdexcept>

#include "drake/common/drake_assert.h"

using Eigen::Dynamic;
using Eigen::Matrix;

template <typename CoefficientType>
ExponentialPlusPiecewisePolynomial<
    CoefficientType>::ExponentialPlusPiecewisePolynomial() {
  // empty
}

template <typename CoefficientType>
ExponentialPlusPiecewisePolynomial<CoefficientType>::
    ExponentialPlusPiecewisePolynomial(
        const PiecewisePolynomial<CoefficientType>& piecewise_polynomial_part)
    : PiecewiseFunction(piecewise_polynomial_part),
      K_(Matrix<CoefficientType, Dynamic, Dynamic>::Zero(
          piecewise_polynomial_part.rows(), 1)),
      A_(Matrix<CoefficientType, Dynamic, Dynamic>::Zero(1, 1)),
      alpha_(Matrix<CoefficientType, Dynamic, Dynamic>::Zero(
          1, piecewise_polynomial_part.getNumberOfSegments())),
      piecewise_polynomial_part_(piecewise_polynomial_part) {
  DRAKE_ASSERT(piecewise_polynomial_part.cols() == 1);
}

template <typename CoefficientType>
typename ExponentialPlusPiecewisePolynomial<CoefficientType>::ValueType
ExponentialPlusPiecewisePolynomial<CoefficientType>::value(double t) const {
  int segment_index = getSegmentIndex(t);

  Eigen::Matrix<double, Eigen::Dynamic, 1> ret =
      piecewise_polynomial_part_.value(t);
  double tj = getStartTime(segment_index);
  auto exponential = (A_ * (t - tj)).eval().exp().eval();
  ret.noalias() += K_ * exponential * alpha_.col(segment_index);
  return ret;
}

template <typename CoefficientType>
ExponentialPlusPiecewisePolynomial<CoefficientType>
ExponentialPlusPiecewisePolynomial<CoefficientType>::derivative(
    int derivative_order) const {
  DRAKE_ASSERT(derivative_order >= 0);
  // quite inefficient, especially for high order derivatives due to all the
  // temporaries...
  MatrixX K_new = K_;
  for (int i = 0; i < derivative_order; i++) {
    K_new = K_new * A_;
  }
  return ExponentialPlusPiecewisePolynomial<CoefficientType>(
      K_new, A_, alpha_,
      piecewise_polynomial_part_.derivative(derivative_order));
}

template <typename CoefficientType>
Eigen::Index ExponentialPlusPiecewisePolynomial<CoefficientType>::rows() const {
  return piecewise_polynomial_part_.rows();
}

template <typename CoefficientType>
Eigen::Index ExponentialPlusPiecewisePolynomial<CoefficientType>::cols() const {
  return piecewise_polynomial_part_.cols();
}

template <typename CoefficientType>
void ExponentialPlusPiecewisePolynomial<CoefficientType>::shiftRight(
    double offset) {
  for (auto it = segment_times.begin(); it != segment_times.end(); ++it) {
    *it += offset;
  }
  piecewise_polynomial_part_.shiftRight(offset);
}

template class DRAKE_EXPORT
    ExponentialPlusPiecewisePolynomial<double>;
