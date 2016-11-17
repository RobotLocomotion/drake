#pragma once

#include <stdexcept>
#include <vector>

#include "drake/solvers/qpSpline/SplineInformation.h"
#include "drake/systems/trajectories/PiecewisePolynomial.h"

class ConstraintMatrixSingularError : public std::runtime_error {
 public:
  ConstraintMatrixSingularError()
      : runtime_error("Constraint matrix is singular!"){}
};

PiecewisePolynomial<double> generateSpline(
    const SplineInformation& spline_information);

PiecewisePolynomial<double> nWaypointCubicSpline(
    const std::vector<double>& segment_times, double x0, double xd0, double xf,
    double xdf, const Eigen::Ref<const Eigen::VectorXd>& xi);
