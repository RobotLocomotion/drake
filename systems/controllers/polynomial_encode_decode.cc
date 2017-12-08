#include "drake/systems/controllers/polynomial_encode_decode.h"

#include <vector>

#include "drake/util/drakeUtil.h"

using Eigen::Dynamic;
using Eigen::Map;
using Eigen::VectorXd;

void encodePolynomial(const Polynomial<double>& polynomial,
                      // NOLINTNEXTLINE(runtime/references)
                      drake::lcmt_polynomial& msg) {
  eigenVectorToStdVector(polynomial.GetCoefficients(), msg.coefficients);
  msg.num_coefficients = polynomial.GetNumberOfCoefficients();
}

Polynomial<double> decodePolynomial(const drake::lcmt_polynomial& msg) {
  Map<const VectorXd> coefficients(msg.coefficients.data(),
                                   msg.coefficients.size());
  return Polynomial<double>(coefficients);
}

void encodePiecewisePolynomial(
    const PiecewisePolynomial<double>& piecewise_polynomial,
    // NOLINTNEXTLINE(runtime/references)
    drake::lcmt_piecewise_polynomial& msg) {
  msg.num_segments = piecewise_polynomial.getNumberOfSegments();
  msg.num_breaks = piecewise_polynomial.getNumberOfSegments() + 1;
  msg.breaks = piecewise_polynomial.getSegmentTimes();
  msg.polynomial_matrices.resize(piecewise_polynomial.getNumberOfSegments());
  for (int i = 0; i < piecewise_polynomial.getNumberOfSegments(); ++i) {
    encodePolynomialMatrix<Eigen::Dynamic, Eigen::Dynamic>(
        piecewise_polynomial.getPolynomialMatrix(i),
        msg.polynomial_matrices[i]);
  }
}

PiecewisePolynomial<double> decodePiecewisePolynomial(
    const drake::lcmt_piecewise_polynomial& msg) {
  typedef PiecewisePolynomial<double>::PolynomialMatrix PolynomialMatrix;
  std::vector<PolynomialMatrix> polynomial_matrices;
  for (size_t i = 0; i < msg.polynomial_matrices.size(); ++i) {
    polynomial_matrices.push_back(
        decodePolynomialMatrix<Dynamic, Dynamic>(msg.polynomial_matrices[i]));
  }
  return PiecewisePolynomial<double>(polynomial_matrices, msg.breaks);
}
