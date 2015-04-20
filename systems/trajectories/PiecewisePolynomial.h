#ifndef DRAKE_SYSTEMS_TRAJECTORIES_PIECEWISEPOLYNOMIAL_H_
#define DRAKE_SYSTEMS_TRAJECTORIES_PIECEWISEPOLYNOMIAL_H_

#include "PiecewisePolynomialBase.h"
#include "Polynomial.h"
#include <vector>

#undef DLLEXPORT
#if defined(WIN32) || defined(WIN64)
  #if defined(drakePiecewisePolynomial_EXPORTS)
    #define DLLEXPORT __declspec( dllexport )
  #else
    #define DLLEXPORT __declspec( dllimport )
  #endif
#else
  #define DLLEXPORT
#endif

class DLLEXPORT PiecewisePolynomial : public PiecewisePolynomialBase
{
private:
  std::vector<Polynomial> polynomials;

  int getSegmentIndex(double t);

public:
  virtual ~PiecewisePolynomial() {};

  PiecewisePolynomial(std::vector<Polynomial> const& polynomials, std::vector<double> const& segment_times);

  PiecewisePolynomial derivative(int derivative_order = 1) const;

  PiecewisePolynomial integral(double value_at_start_time = 0.0) const;

  double value(double t);

  const Polynomial& getPolynomial(int segment_index) const;

  virtual int getSegmentPolynomialDegree(int segment_index) const;

  PiecewisePolynomial& operator+=(const PiecewisePolynomial& other);

  PiecewisePolynomial& operator*=(const PiecewisePolynomial& other);

  const PiecewisePolynomial operator+(const PiecewisePolynomial &other) const;

  const PiecewisePolynomial operator*(const PiecewisePolynomial &other) const;

  bool isApprox(const PiecewisePolynomial& other, double tol) const;

protected:
  PiecewisePolynomial();

  double segmentValueAtGlobalAbscissa(int segment_index, double t) const;
};

#endif /* DRAKE_SYSTEMS_TRAJECTORIES_PIECEWISEPOLYNOMIAL_H_ */
