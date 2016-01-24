#ifndef DRAKE_SOLVERS_QPSPLINE_SPLINEGENERATION_H_
#define DRAKE_SOLVERS_QPSPLINE_SPLINEGENERATION_H_

#include "SplineInformation.h"
#include "drake/systems/trajectories/PiecewisePolynomial.h"
#include <stdexcept>
#include "drake/drakeSplineGeneration_export.h"

class ConstraintMatrixSingularError : public std::runtime_error {
public:
  ConstraintMatrixSingularError() : runtime_error("Constraint matrix is singular!") { };
};

DRAKESPLINEGENERATION_EXPORT PiecewisePolynomial<double> generateSpline(const SplineInformation& spline_information);

DRAKESPLINEGENERATION_EXPORT PiecewisePolynomial<double> nWaypointCubicSpline(const std::vector<double>& segment_times, double x0, double xd0, double xf, double xdf, const Eigen::Ref<const Eigen::VectorXd> &xi);

#endif /* DRAKE_SOLVERS_QPSPLINE_SPLINEGENERATION_H_ */
