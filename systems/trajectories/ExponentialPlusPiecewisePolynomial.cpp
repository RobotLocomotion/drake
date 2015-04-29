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
ExponentialPlusPiecewisePolynomial<CoefficientType>::ExponentialPlusPiecewisePolynomial(
    const MatrixX& K, const MatrixX& A, const std::vector<VectorX>& alpha,
    const PiecewisePolynomial<CoefficientType>& piecewise_polynomial_part) :
    PiecewiseFunction(piecewise_polynomial_part), K(K), A(A), alpha(alpha), piecewise_polynomial_part(piecewise_polynomial_part)
{
  int num_segments = piecewise_polynomial_part.getNumberOfSegments();
  assert(A.rows() == rows());
  assert(A.cols() == rows());
  assert(alpha.size() == num_segments);
  for (int i = 0; i < num_segments; i++)
    assert(alpha[i].rows() == rows());
  assert(piecewise_polynomial_part.rows() == rows());
  assert(piecewise_polynomial_part.cols() == 1);
}

template<typename CoefficientType>
typename ExponentialPlusPiecewisePolynomial<CoefficientType>::ValueType ExponentialPlusPiecewisePolynomial<CoefficientType>::value(double t)
{
  int segment_index = getSegmentIndex(t);

  Eigen::Matrix<double, Eigen::Dynamic, 1> ret = piecewise_polynomial_part.value(t);
  double tj = getStartTime(segment_index);
  auto exponential = (A * (t - tj)).eval().exp().eval();
  ret.noalias() += K * exponential * alpha[segment_index];
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
  return ExponentialPlusPiecewisePolynomial(K_new, A, alpha, piecewise_polynomial_part.derivative(derivative_order));
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
