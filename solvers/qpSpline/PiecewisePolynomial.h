#ifndef DRAKE_SOLVERS_QPSPLINE_PIECEWISEPOLYNOMIAL_H_
#define DRAKE_SOLVERS_QPSPLINE_PIECEWISEPOLYNOMIAL_H_

#include "PiecewisePolynomialBase.h"
#include "Polynomial.h"
#include <vector>

class PiecewisePolynomial : public PiecewisePolynomialBase
{
private:
  std::vector<Polynomial> polynomials;
  std::vector<double> segment_times;

  int getSegmentIndex(double t);

public:
  PiecewisePolynomial(std::vector<Polynomial> const& polynomials, std::vector<double> const& segment_times);

  PiecewisePolynomial derivative(int derivative_order = 1) const;

  PiecewisePolynomial integral(double integration_constant) const;

  double value(double t);

  virtual int getSegmentPolynomialOrder(int segment_number) const;
};

#endif /* DRAKE_SOLVERS_QPSPLINE_PIECEWISEPOLYNOMIAL_H_ */
