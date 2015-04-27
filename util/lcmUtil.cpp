#include "lcmUtil.h"
#include <Eigen/Core>
#include "drakeUtil.h"

using namespace Eigen;

void encodePolynomial(const Polynomial<double>& polynomial, drake::lcmt_polynomial& msg)
{
  eigenVectorToStdVector(polynomial.getCoefficients(), msg.coefficients);
  msg.num_coefficients = polynomial.getNumberOfCoefficients();
}

Polynomial<double> decodePolynomial(drake::lcmt_polynomial& msg)
{
  Map<VectorXd> coefficients(msg.coefficients.data(), msg.coefficients.size());
  return Polynomial<double>(coefficients);
}
