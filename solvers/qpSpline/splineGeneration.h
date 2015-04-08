#ifndef DRAKE_SOLVERS_QPSPLINE_SPLINEGENERATION_H_
#define DRAKE_SOLVERS_QPSPLINE_SPLINEGENERATION_H_

#include "SplineInformation.h"
#include "PiecewisePolynomial.h"
#include <stdexcept>

class ConstraintMatrixSingularError : public std::runtime_error {
public:
  ConstraintMatrixSingularError() : runtime_error("Constraint matrix is singular!") { };
};

PiecewisePolynomial generateSpline(const SplineInformation& spline_information);

PiecewisePolynomial twoWaypointCubicSpline(const std::vector<double>& segment_times, double x0, double xd0, double xf, double xdf, double x1, double x2);


#endif /* DRAKE_SOLVERS_QPSPLINE_SPLINEGENERATION_H_ */
