#include "ExponentialPlusPiecewisePolynomial.h"
#include <unsupported/Eigen/MatrixFunctions>
#include <stdexcept>

using namespace Eigen;

template<typename CoefficientType>
ExponentialPlusPiecewisePolynomial<CoefficientType>::ExponentialPlusPiecewisePolynomial()
{
  // empty
}

template<typename CoefficientType>
ExponentialPlusPiecewisePolynomial<CoefficientType>::ExponentialPlusPiecewisePolynomial(const PiecewisePolynomial<CoefficientType>& piecewise_polynomial_part) :
    PiecewiseFunction(piecewise_polynomial_part),
    K(Matrix<CoefficientType, Dynamic, Dynamic>::Zero(piecewise_polynomial_part.rows(), 1)),
    A(Matrix<CoefficientType, Dynamic, Dynamic>::Zero(1, 1)),
    alpha(Matrix<CoefficientType, Dynamic, Dynamic>::Zero(1, piecewise_polynomial_part.getNumberOfSegments())),
    piecewise_polynomial_part(piecewise_polynomial_part)
{
  assert(piecewise_polynomial_part.cols() == 1);
}

template<typename CoefficientType>
typename ExponentialPlusPiecewisePolynomial<CoefficientType>::ValueType ExponentialPlusPiecewisePolynomial<CoefficientType>::value(double t) const
{
  int segment_index = getSegmentIndex(t);

  Eigen::Matrix<double, Eigen::Dynamic, 1> ret = piecewise_polynomial_part.value(t);
  double tj = getStartTime(segment_index);
  auto exponential = (A * (t - tj)).eval().exp().eval();
  ret.noalias() += K * exponential * alpha.col(segment_index);
  return ret;
}

template<typename CoefficientType>
ExponentialPlusPiecewisePolynomial<CoefficientType> ExponentialPlusPiecewisePolynomial<CoefficientType>::derivative(int derivative_order) const
{
  assert(derivative_order >= 0);
  // quite inefficient, especially for high order derivatives due to all the temporaries...
  MatrixX K_new = K;
  for (int i = 0; i < derivative_order; i++) {
    K_new = K_new * A;
  }
  return ExponentialPlusPiecewisePolynomial<CoefficientType>(K_new, A, alpha, piecewise_polynomial_part.derivative(derivative_order));
}


template<typename CoefficientType>
Eigen::DenseIndex ExponentialPlusPiecewisePolynomial<CoefficientType>::rows() const
{
  return piecewise_polynomial_part.rows();
}

template<typename CoefficientType>
Eigen::DenseIndex ExponentialPlusPiecewisePolynomial<CoefficientType>::cols() const
{
  return piecewise_polynomial_part.cols();
}

template<typename CoefficientType>
void ExponentialPlusPiecewisePolynomial<CoefficientType>::shiftRight(double offset) {
  for (auto it = segment_times.begin(); it != segment_times.end(); ++it) {
    *it += offset;
  }
  piecewise_polynomial_part.shiftRight(offset);
}

template class DLLEXPORT ExponentialPlusPiecewisePolynomial<double>;
