#include "ExponentialPlusPiecewisePolynomial.h"
#include <unsupported/Eigen/MatrixFunctions>
#include <stdexcept>

using namespace Eigen;

template<typename CoefficientType>
ExponentialPlusPiecewisePolynomial<CoefficientType>::ExponentialPlusPiecewisePolynomial(
    const Eigen::Ref<MatrixX>& K, const Eigen::Ref<MatrixX>& A, const std::vector<VectorX>& alpha,
    const PiecewisePolynomial<CoefficientType>& piecewise_polynomial_part) :
    K(K), A(A), alpha(alpha), piecewise_polynomial_part(piecewise_polynomial_part)
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
  int segment_index = piecewise_polynomial_part.getSegmentIndex(t);

  Eigen::Matrix<double, Eigen::Dynamic, 1> ret = piecewise_polynomial_part.matrixValue(t);
  double tj = piecewise_polynomial_part.getStartTime(segment_index);
  auto exponential = (A * (t - tj)).eval().exp().eval();
  ret.noalias() += K * exponential * alpha[segment_index];
  return ret;
}

template<typename CoefficientType>
Eigen::DenseIndex ExponentialPlusPiecewisePolynomial<CoefficientType>::rows()
{
  return piecewise_polynomial_part.rows();
}

template class DLLEXPORT ExponentialPlusPiecewisePolynomial<double>;
