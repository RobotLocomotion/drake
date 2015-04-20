#include "Polynomial.h"
#include <cassert>
#include <algorithm>
#include <cmath>

using namespace std;
using namespace Eigen;

Polynomial::Polynomial(Eigen::Ref<Eigen::VectorXd> const& coefficients) :
  coefficients(coefficients)
{
  assert(coefficients.rows() > 0);
}

Polynomial::Polynomial(int num_coefficients) :
  coefficients(num_coefficients)
{
  // empty
}

int Polynomial::getNumberOfCoefficients() const {
  return static_cast<int>(coefficients.size());
}

int Polynomial::getDegree() const {
  return getNumberOfCoefficients() - 1;
}

Eigen::VectorXd const& Polynomial::getCoefficients() const {
  return coefficients;
}

Polynomial Polynomial::derivative(int derivative_order) const {
  assert(derivative_order >= 0);
  int derivative_num_coefficients = getNumberOfCoefficients() - derivative_order;
  if (derivative_num_coefficients <= 0)
    return Polynomial::zero();

  Polynomial ret(derivative_num_coefficients);
  for (int i = 0; i < ret.getNumberOfCoefficients(); i++) {
    CoefficientType factorial = 1.0;
    for (int j = 0; j < derivative_order; j++)
    {
      factorial *= i + derivative_order - j;
    }
    ret.coefficients(i) = factorial * coefficients(i + derivative_order);
  }
  return ret;
}

Polynomial Polynomial::integral(CoefficientType integration_constant) const {
  Polynomial ret(getNumberOfCoefficients() + 1);
  ret.coefficients(0) = integration_constant;
  for (int i = 1; i < ret.getNumberOfCoefficients(); i++) {
    ret.coefficients(i) = coefficients(i - 1) / i;
  }
  return ret;
}

Polynomial& Polynomial::operator+=(const Polynomial& other) {
  int old_num_coefficients = getNumberOfCoefficients();
  int new_num_coefficients = std::max(old_num_coefficients, other.getNumberOfCoefficients());
  coefficients.conservativeResize(new_num_coefficients);
  coefficients.tail(new_num_coefficients - old_num_coefficients).setZero();
  coefficients.head(other.getNumberOfCoefficients()) += other.coefficients;
  return *this;
}

Polynomial& Polynomial::operator-=(const Polynomial& other) {
  int old_num_coefficients = getNumberOfCoefficients();
  int new_num_coefficients = std::max(old_num_coefficients, other.getNumberOfCoefficients());
  coefficients.conservativeResize(new_num_coefficients);
  coefficients.tail(new_num_coefficients - old_num_coefficients).setZero();
  coefficients.head(other.getNumberOfCoefficients()) -= other.coefficients;
  return *this;
}

Polynomial& Polynomial::operator*=(const Polynomial& other) {
  // TODO: more efficient implementation
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

Polynomial& Polynomial::operator+=(const CoefficientType& scalar) {
  coefficients[0] += scalar;
  return *this;
}

Polynomial& Polynomial::operator-=(const CoefficientType& scalar) {
  coefficients[0] -= scalar;
  return *this;
}

Polynomial& Polynomial::operator*=(const CoefficientType& scalar) {
  coefficients *= scalar;
  return *this;
}

Polynomial& Polynomial::operator/=(const CoefficientType& scalar) {
  coefficients /= scalar;
  return *this;
}

const Polynomial Polynomial::operator+(const Polynomial& other) const {
  Polynomial ret = *this;
  ret += other;
  return ret;
}

const Polynomial Polynomial::operator-(const Polynomial& other) const {
  Polynomial ret = *this;
  ret -= other;
  return ret;
}

const Polynomial Polynomial::operator*(const Polynomial& other) const {
  Polynomial ret = *this;
  ret *= other;
  return ret;
}

const Polynomial Polynomial::operator+(const CoefficientType& scalar) const {
  Polynomial ret = *this;
  ret += scalar;
  return ret;
}

const Polynomial Polynomial::operator-(const CoefficientType& scalar) const {
  Polynomial ret = *this;
  ret -= scalar;
  return ret;
}

const Polynomial Polynomial::operator*(const CoefficientType& scalar) const {
  Polynomial ret = *this;
  ret *= scalar;
  return ret;
}

const Polynomial Polynomial::operator/(const CoefficientType& scalar) const {
  Polynomial ret = *this;
  ret /= scalar;
  return ret;
}

Polynomial::RootsType Polynomial::roots() const {
  // need to handle degree 0 and 1 explicitly because Eigen's polynomial solver doesn't work for these
  int degree = getDegree();
  switch (degree) {
  case 0:
    return Polynomial::RootsType(degree);
  case 1: {
    Polynomial::RootsType ret(degree);
    ret[0] = -coefficients[0] / coefficients[1];
    return ret;
  }
  default: {
    PolynomialSolver<CoefficientType, Eigen::Dynamic> solver;
    solver.compute(coefficients);
    return solver.roots();
    break;
  }
  }
}

bool Polynomial::isApprox(const Polynomial& other, const CoefficientType& tol) const {
  return coefficients.isApprox(other.coefficients, tol);
}

Polynomial Polynomial::zero() {
  Polynomial ret(1);
  ret.coefficients(0) = 0.0;
  return ret;
}
