#pragma once

#include "SplineInformation.h"
#include "drake/systems/trajectories/PiecewisePolynomial.h"
#include <stdexcept>
#include "drake/common/drake_export.h"

class ConstraintMatrixSingularError : public std::runtime_error {
 public:
  ConstraintMatrixSingularError()
      : runtime_error("Constraint matrix is singular!"){}
};

DRAKE_EXPORT PiecewisePolynomial<double> generateSpline(
    const SplineInformation& spline_information);

DRAKE_EXPORT PiecewisePolynomial<double> nWaypointCubicSpline(
    const std::vector<double>& segment_times, double x0, double xd0, double xf,
    double xdf, const Eigen::Ref<const Eigen::VectorXd>& xi);
