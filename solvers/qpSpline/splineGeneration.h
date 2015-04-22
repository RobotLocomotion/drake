#ifndef DRAKE_SOLVERS_QPSPLINE_SPLINEGENERATION_H_
#define DRAKE_SOLVERS_QPSPLINE_SPLINEGENERATION_H_

#include "SplineInformation.h"
#include "PiecewisePolynomial.h"
#include <stdexcept>

#undef DLLEXPORT
#if defined(WIN32) || defined(WIN64)
  #if defined(drakeSplineGeneration_EXPORTS)
    #define DLLEXPORT __declspec( dllexport )
  #else
    #define DLLEXPORT __declspec( dllimport )
  #endif
#else
  #define DLLEXPORT
#endif

class ConstraintMatrixSingularError : public std::runtime_error {
public:
  ConstraintMatrixSingularError() : runtime_error("Constraint matrix is singular!") { };
};

DLLEXPORT PiecewisePolynomial generateSpline(const SplineInformation& spline_information);

DLLEXPORT PiecewisePolynomial nWaypointCubicSpline(const std::vector<double>& segment_times, double x0, double xd0, double xf, double xdf, const std::vector<double>& xi);


#endif /* DRAKE_SOLVERS_QPSPLINE_SPLINEGENERATION_H_ */
