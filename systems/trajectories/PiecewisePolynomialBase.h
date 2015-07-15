#ifndef DRAKE_SYSTEMS_TRAJECTORIES_PIECEWISEPOLYNOMIALBASE_H_
#define DRAKE_SYSTEMS_TRAJECTORIES_PIECEWISEPOLYNOMIALBASE_H_

#include <Eigen/Core>
#include <vector>
#include "PiecewiseFunction.h"

#undef DLLEXPORT
#if defined(WIN32) || defined(WIN64)
  #if defined(drakeTrajectories_EXPORTS)
    #define DLLEXPORT __declspec( dllexport )
  #else
    #define DLLEXPORT __declspec( dllimport )
  #endif
#else
  #define DLLEXPORT
#endif

class DLLEXPORT PiecewisePolynomialBase : public PiecewiseFunction
{
public:
  PiecewisePolynomialBase(std::vector<double> const & segment_times);

  virtual ~PiecewisePolynomialBase();

  virtual int getSegmentPolynomialDegree(int segment_number, Eigen::DenseIndex row = 0, Eigen::DenseIndex col = 0) const = 0;

  int getNumberOfCoefficients(int segment_number, Eigen::DenseIndex row = 0, Eigen::DenseIndex col = 0) const;

  int getTotalNumberOfCoefficients(Eigen::DenseIndex row = 0, Eigen::DenseIndex col = 0) const;

protected:
  PiecewisePolynomialBase();
};

#endif /* DRAKE_SYSTEMS_TRAJECTORIES_PIECEWISEPOLYNOMIALBASE_H_ */
