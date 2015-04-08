#include "Polynomial.h"
#include <cassert>

Polynomial::Polynomial(Eigen::Ref<Eigen::VectorXd> const& coefficients) :
  coefficients(coefficients)
{
  // empty
}

int Polynomial::getOrder() const {
  return static_cast<int>(coefficients.size() - 1);
}

double Polynomial::value(double t) const {
  return derivativeValue(0, t);
}

double Polynomial::derivativeValue(int derivative_order, double t) const {
  assert(derivative_order >= 0);

  double ret = 0.0;
  double t_power = 1.0;

  for (int coefficient_index = derivative_order; coefficient_index < coefficients.rows(); coefficient_index++)
  {
     double factorial = 1.0;
     for (int i = 0; i< derivative_order; i++)
     {
        factorial *= (coefficient_index - i);
     }
     ret += factorial * coefficients(coefficient_index) * t_power;
     t_power *= t;
  }
  return ret;
}
