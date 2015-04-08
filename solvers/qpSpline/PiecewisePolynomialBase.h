#ifndef DRAKE_SOLVERS_QPSPLINE_PIECEWISEPOLYNOMIALBASE_H_
#define DRAKE_SOLVERS_QPSPLINE_PIECEWISEPOLYNOMIALBASE_H_

#include <vector>

class PiecewisePolynomialBase
{
protected:
  std::vector<double> segment_times;

public:
  PiecewisePolynomialBase(std::vector<double> const & segment_times);

  virtual ~PiecewisePolynomialBase();

  virtual int getSegmentPolynomialOrder(int segment_number) const = 0;

  int getNumberOfSegments() const;

  int getNumberOfCoefficients(int segment_number) const;

  double getStartTime(int segment_number) const;

  double getEndTime(int segment_number) const;

  double getStartTime() const;

  double getEndTime() const;

  int getTotalNumberOfCoefficients() const;
};

#endif /* DRAKE_SOLVERS_QPSPLINE_PIECEWISEPOLYNOMIALBASE_H_ */
