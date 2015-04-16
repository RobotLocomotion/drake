#include "Polynomial.h"
#include <cassert>
#include <algorithm>
#include <cmath>

using namespace std;
using namespace Eigen;

Polynomial::Polynomial(Eigen::Ref<Eigen::VectorXd> const& coefficients) :
  coefficients(coefficients)
{
  // empty
}

Polynomial::Polynomial(int num_coefficients) :
  coefficients(num_coefficients)
{
  // empty
}

int Polynomial::getNumberOfCoefficients() const {
  return static_cast<int>(coefficients.size());
}

int Polynomial::getOrder() const {
  return getNumberOfCoefficients() - 1;
}

Eigen::VectorXd const& Polynomial::getCoefficients() const {
  return coefficients;
}

double Polynomial::valueHorner(double t) const {
  // Horner's method
  double ret = coefficients(getNumberOfCoefficients() - 1);
  for (int i = getNumberOfCoefficients() - 2; i >= 0; --i) {
    ret = ret * t + coefficients[i];
  }
  return ret;
}

double Polynomial::valueStabilizedHorner(double t) const {
  // stabilized Horner
  double val = coefficients[0];
  double inv_t = 1.0 / t;
  for (DenseIndex i = 1; i < coefficients.size(); ++i) {
    val = val * inv_t + coefficients[i];
  }

  return std::pow(t, (double) (coefficients.size() - 1)) * val;
}

double Polynomial::value(double t) const {
  // stolen from Eigen/unsupported PolynomialUtils
  if (abs(t) <= 1.0 ) {
    return valueHorner(t);
  }
  else {
    return valueStabilizedHorner(t);
  }
}

Polynomial Polynomial::derivative(int derivative_order) const {
  assert(derivative_order >= 0);
  int derivative_num_coefficients = getNumberOfCoefficients() - derivative_order;
  if (derivative_num_coefficients <= 0)
    return Polynomial::zero();

  Polynomial ret(derivative_num_coefficients);
  for (int i = 0; i < ret.getNumberOfCoefficients(); i++) {
    double factorial = 1.0;
    for (int j = 0; j < derivative_order; j++)
    {
      factorial *= i + derivative_order - j;
    }
    ret.coefficients(i) = factorial * coefficients(i + derivative_order);
  }
  return ret;
}

Polynomial Polynomial::integral(double integration_constant) const {
  Polynomial ret(getNumberOfCoefficients() + 1);
  ret.coefficients(0) = integration_constant;
  for (int i = 1; i < ret.getNumberOfCoefficients(); i++) {
    ret.coefficients(i) = coefficients(i - 1) / i;
  }
  return ret;
}

Polynomial& Polynomial::operator+=(const Polynomial& other) {
  int old_num_coefficients = getNumberOfCoefficients();
  int new_num_coefficients = max(old_num_coefficients, other.getNumberOfCoefficients());
  coefficients.conservativeResize(new_num_coefficients);
  coefficients.segment(old_num_coefficients, new_num_coefficients - old_num_coefficients).setZero();
  coefficients += other.coefficients;
  return *this;
}

Polynomial& Polynomial::operator*=(const Polynomial& other) {
  // TODO: better implementation
  MatrixXd outer_product = coefficients * other.coefficients.transpose();
  coefficients.setZero(outer_product.rows() + outer_product.cols() - 1);
  for (DenseIndex i = 0; i < getNumberOfCoefficients(); i++) {
    for (DenseIndex row = max(i - outer_product.cols() + 1, (DenseIndex) 0); row < min(i + 1, outer_product.rows()); row++) {
      DenseIndex col = i - row;
      coefficients(i) += outer_product(row, col);
    }
  }
  return *this;
}

const Polynomial Polynomial::operator+(const Polynomial &other) const {
  Polynomial ret = *this;
  ret += other;
  return ret;
}

const Polynomial Polynomial::operator*(const Polynomial &other) const {
  Polynomial ret = *this;
  ret *= other;
  return ret;
}

bool Polynomial::isApprox(const Polynomial& other, double tol) const {
  return coefficients.isApprox(other.coefficients, tol);
}

Polynomial Polynomial::zero() {
  Polynomial ret(1);
  ret.coefficients(0) = 0.0;
  return ret;
}
