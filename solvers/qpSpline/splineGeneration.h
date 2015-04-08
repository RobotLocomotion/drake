#ifndef DRAKE_SOLVERS_QPSPLINE_SPLINEGENERATION_H_
#define DRAKE_SOLVERS_QPSPLINE_SPLINEGENERATION_H_

#include "SplineInformation.h"
#include "PiecewisePolynomial.h"

PiecewisePolynomial generateSpline(const SplineInformation& spline_information);


#endif /* DRAKE_SOLVERS_QPSPLINE_SPLINEGENERATION_H_ */
