#include "lcmUtil.h"
#include <Eigen/Core>
#include "drakeUtil.h"

using namespace Eigen;

void encodePolynomial(const Polynomial<double>& polynomial, drake::lcmt_polynomial& msg)
{
  eigenVectorToStdVector(polynomial.getCoefficients(), msg.coefficients);
  msg.num_coefficients = polynomial.getNumberOfCoefficients();
}

Polynomial<double> decodePolynomial(const drake::lcmt_polynomial& msg)
{
  Map<const VectorXd> coefficients(msg.coefficients.data(), msg.coefficients.size());
  return Polynomial<double>(coefficients);
}

void encodePiecewisePolynomial(const PiecewisePolynomial<double>& piecewise_polynomial, drake::lcmt_piecewise_polynomial& msg)
{
  msg.num_segments = piecewise_polynomial.getNumberOfSegments();
  msg.num_breaks = piecewise_polynomial.getNumberOfSegments() + 1;
  msg.breaks = piecewise_polynomial.getSegmentTimes();
  msg.polynomial_matrices.resize(piecewise_polynomial.getNumberOfSegments());
  for (int i = 0; i < piecewise_polynomial.getNumberOfSegments(); ++i) {
    encodePolynomialMatrix(piecewise_polynomial.getPolynomialMatrix(i), msg.polynomial_matrices[i]);
  }
}

PiecewisePolynomial<double> decodePiecewisePolynomial(const drake::lcmt_piecewise_polynomial& msg)
{
  typedef PiecewisePolynomial<double>::PolynomialMatrix PolynomialMatrix;
  std::vector<PolynomialMatrix> polynomial_matrices;
  for (int i = 0; i < msg.polynomial_matrices.size(); ++i) {
    polynomial_matrices.push_back(decodePolynomialMatrix<Dynamic, Dynamic>(msg.polynomial_matrices[i]));
  }
  return PiecewisePolynomial<double>(polynomial_matrices, msg.breaks);
}

