#ifndef DRAKE_SOLVERS_QPSPLINE_PIECEWISEPOLYNOMIALBASE_H_
#define DRAKE_SOLVERS_QPSPLINE_PIECEWISEPOLYNOMIALBASE_H_

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

class DLLEXPORT PiecewisePolynomialBase
{
protected:
  std::vector<double> segment_times;

public:
  PiecewisePolynomialBase(std::vector<double> const & segment_times);

  virtual ~PiecewisePolynomialBase();

  virtual int getSegmentPolynomialDegree(int segment_number) const = 0;

  int getNumberOfSegments() const;

  int getNumberOfCoefficients(int segment_number) const;

  double getStartTime(int segment_number) const;

  double getEndTime(int segment_number) const;

  double getDuration(int segment_number) const;

  double getStartTime() const;

  double getEndTime() const;

  int getTotalNumberOfCoefficients() const;

protected:
  void segmentNumberRangeCheck(int segment_number) const;

  bool segmentTimesEqual(const PiecewisePolynomialBase& b, double tol) const;

  PiecewisePolynomialBase();
};

#endif /* DRAKE_SOLVERS_QPSPLINE_PIECEWISEPOLYNOMIALBASE_H_ */
