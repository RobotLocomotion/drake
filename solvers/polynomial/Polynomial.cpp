#include "Polynomial.h"
#include <cassert>
#include <algorithm>
#include <cmath>

using namespace std;
using namespace Eigen;

template <typename CoefficientType>
Polynomial<CoefficientType>::Polynomial(Eigen::Ref<CoefficientsType> const& coefficients) :
  coefficients(coefficients)
{
  assert(coefficients.rows() > 0);
}

template <typename CoefficientType>
Polynomial<CoefficientType>::Polynomial(int num_coefficients) :
  coefficients(num_coefficients)
{
  assert(coefficients.rows() > 0);
  // empty;
}

template <typename CoefficientType>
int Polynomial<CoefficientType>::getNumberOfCoefficients() const {
  return static_cast<int>(coefficients.size());
}

template <typename CoefficientType>
int Polynomial<CoefficientType>::getDegree() const {
  return getNumberOfCoefficients() - 1;
}

template <typename CoefficientType>
typename Polynomial<CoefficientType>::CoefficientsType const& Polynomial<CoefficientType>::getCoefficients() const {
  return coefficients;
}

template <typename CoefficientType>
Polynomial<CoefficientType> Polynomial<CoefficientType>::derivative(int derivative_order) const {
  assert(derivative_order >= 0);
  int derivative_num_coefficients = getNumberOfCoefficients() - derivative_order;
  if (derivative_num_coefficients <= 0)
    return Polynomial<CoefficientType>::zero();

  Polynomial<CoefficientType> ret(derivative_num_coefficients);
  for (int i = 0; i < ret.getNumberOfCoefficients(); i++) {
    RealScalar factorial = 1.0;
    for (int j = 0; j < derivative_order; j++)
    {
      factorial *= i + derivative_order - j;
    }
    ret.coefficients(i) = factorial * coefficients(i + derivative_order);
  }
  return ret;
}

template <typename CoefficientType>
Polynomial<CoefficientType> Polynomial<CoefficientType>::integral(const CoefficientType& integration_constant) const {
  Polynomial<CoefficientType> ret(getNumberOfCoefficients() + 1);
  ret.coefficients(0) = integration_constant;
  for (int i = 1; i < ret.getNumberOfCoefficients(); i++) {
    ret.coefficients(i) = coefficients(i - 1) / (RealScalar) i;
  }
  return ret;
}

template <typename CoefficientType>
Polynomial<CoefficientType>& Polynomial<CoefficientType>::operator+=(const Polynomial<CoefficientType>& other) {
  int old_num_coefficients = getNumberOfCoefficients();
  int new_num_coefficients = std::max(old_num_coefficients, other.getNumberOfCoefficients());
  coefficients.conservativeResize(new_num_coefficients);
  coefficients.tail(new_num_coefficients - old_num_coefficients).setZero();
  coefficients.head(other.getNumberOfCoefficients()) += other.coefficients;
  return *this;
}

template <typename CoefficientType>
Polynomial<CoefficientType>& Polynomial<CoefficientType>::operator-=(const Polynomial<CoefficientType>& other) {
  int old_num_coefficients = getNumberOfCoefficients();
  int new_num_coefficients = std::max(old_num_coefficients, other.getNumberOfCoefficients());
  coefficients.conservativeResize(new_num_coefficients);
  coefficients.tail(new_num_coefficients - old_num_coefficients).setZero();
  coefficients.head(other.getNumberOfCoefficients()) -= other.coefficients;
  return *this;
}

template <typename CoefficientType>
Polynomial<CoefficientType>& Polynomial<CoefficientType>::operator*=(const Polynomial<CoefficientType>& other) {
  // TODO: more efficient implementation
  auto outer_product = (coefficients * other.coefficients.transpose()).eval();
  coefficients.setZero(outer_product.rows() + outer_product.cols() - 1);
  for (DenseIndex i = 0; i < getNumberOfCoefficients(); i++) {
    for (DenseIndex row = max(i - outer_product.cols() + 1, (DenseIndex) 0); row < min(i + 1, outer_product.rows()); row++) {
      DenseIndex col = i - row;
      coefficients(i) += outer_product(row, col);
    }
  }
  return *this;
}

template <typename CoefficientType>
Polynomial<CoefficientType>& Polynomial<CoefficientType>::operator+=(const CoefficientType& scalar) {
  coefficients[0] += scalar;
  return *this;
}

template <typename CoefficientType>
Polynomial<CoefficientType>& Polynomial<CoefficientType>::operator-=(const CoefficientType& scalar) {
  coefficients[0] -= scalar;
  return *this;
}

template <typename CoefficientType>
Polynomial<CoefficientType>& Polynomial<CoefficientType>::operator*=(const CoefficientType& scalar) {
  coefficients *= scalar;
  return *this;
}

template <typename CoefficientType>
Polynomial<CoefficientType>& Polynomial<CoefficientType>::operator/=(const CoefficientType& scalar) {
  coefficients /= scalar;
  return *this;
}

template <typename CoefficientType>
const Polynomial<CoefficientType> Polynomial<CoefficientType>::operator+(const Polynomial& other) const {
  Polynomial<CoefficientType> ret = *this;
  ret += other;
  return ret;
}

template <typename CoefficientType>
const Polynomial<CoefficientType> Polynomial<CoefficientType>::operator-(const Polynomial& other) const {
  Polynomial<CoefficientType> ret = *this;
  ret -= other;
  return ret;
}

template <typename CoefficientType>
const Polynomial<CoefficientType> Polynomial<CoefficientType>::operator*(const Polynomial& other) const {
  Polynomial<CoefficientType> ret = *this;
  ret *= other;
  return ret;
}

template <typename CoefficientType>
const Polynomial<CoefficientType> Polynomial<CoefficientType>::operator+(const CoefficientType& scalar) const {
  Polynomial<CoefficientType> ret = *this;
  ret += scalar;
  return ret;
}

template <typename CoefficientType>
const Polynomial<CoefficientType> Polynomial<CoefficientType>::operator-(const CoefficientType& scalar) const {
  Polynomial<CoefficientType> ret = *this;
  ret -= scalar;
  return ret;
}

template <typename CoefficientType>
const Polynomial<CoefficientType> Polynomial<CoefficientType>::operator*(const CoefficientType& scalar) const {
  Polynomial<CoefficientType> ret = *this;
  ret *= scalar;
  return ret;
}

template <typename CoefficientType>
const Polynomial<CoefficientType> Polynomial<CoefficientType>::operator/(const CoefficientType& scalar) const {
  Polynomial<CoefficientType> ret = *this;
  ret /= scalar;
  return ret;
}

template <typename CoefficientType>
typename Polynomial<CoefficientType>::RootsType Polynomial<CoefficientType>::roots() const {
  // need to handle degree 0 and 1 explicitly because Eigen's polynomial solver doesn't work for these
  int degree = getDegree();
  switch (degree) {
  case 0:
    return Polynomial<CoefficientType>::RootsType(degree);
  case 1: {
    Polynomial<CoefficientType>::RootsType ret(degree);
    ret[0] = -coefficients[0] / coefficients[1];
    return ret;
  }
  default: {
    PolynomialSolver<RealScalar, Eigen::Dynamic> solver;
    solver.compute(coefficients);
    return solver.roots();
    break;
  }
  }
}

template <typename CoefficientType>
bool Polynomial<CoefficientType>::isApprox(const Polynomial& other, const RealScalar& tol) const {
  return coefficients.isApprox(other.coefficients, tol);
}

template <typename CoefficientType>
Polynomial<CoefficientType> Polynomial<CoefficientType>::zero() {
  Polynomial<CoefficientType> ret(1);
  ret.coefficients(0) = (CoefficientType) 0;
  return ret;
}

template class DLLEXPORT Polynomial<double>;
//template class DLLEXPORT Polynomial<std::complex<double>>; // doesn't work yet because the roots solver can't handle it
