#ifndef DRAKE_SOLVERS_QPSPLINE_PIECEWISEPOLYNOMIAL_H_
#define DRAKE_SOLVERS_QPSPLINE_PIECEWISEPOLYNOMIAL_H_

#include "PiecewisePolynomialBase.h"
#include "Polynomial.h"
#include <vector>

class PiecewisePolynomial : public PiecewisePolynomialBase
{
private:
  std::vector<Polynomial> polynomials;

  int getSegmentIndex(double t);

public:
  virtual ~PiecewisePolynomial() {};

  PiecewisePolynomial(std::vector<Polynomial> const& polynomials, std::vector<double> const& segment_times);

  PiecewisePolynomial derivative(int derivative_order = 1) const;

  PiecewisePolynomial integral(double integration_constant = 0.0) const;

  double value(double t);

  const Polynomial& getPolynomial(int segment_index) const;

  virtual int getSegmentPolynomialOrder(int segment_number) const;

  PiecewisePolynomial& operator+=(const PiecewisePolynomial& other);

  PiecewisePolynomial& operator*=(const PiecewisePolynomial& other);

  const PiecewisePolynomial operator+(const PiecewisePolynomial &other) const;

  const PiecewisePolynomial operator*(const PiecewisePolynomial &other) const;
};

#endif /* DRAKE_SOLVERS_QPSPLINE_PIECEWISEPOLYNOMIAL_H_ */
