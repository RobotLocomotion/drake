#ifndef DRAKE_SOLVERS_QPSPLINE_POLYNOMIAL_H_
#define DRAKE_SOLVERS_QPSPLINE_POLYNOMIAL_H_

#include <Eigen/Core>

class Polynomial
{
private:
  Eigen::VectorXd coefficients;

public:
  Polynomial(Eigen::Ref<Eigen::VectorXd> const& coefficients);

  int getOrder() const;

  double value(double t) const;

  double derivativeValue(int derivative_order, double t) const;
};

#endif /* DRAKE_SOLVERS_QPSPLINE_POLYNOMIAL_H_ */
