#pragma once

#include <stdexcept>
#include <vector>

#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/common/trajectories/qp_spline/spline_information.h"

class ConstraintMatrixSingularError : public std::runtime_error {
 public:
  ConstraintMatrixSingularError()
      : runtime_error("Constraint matrix is singular!"){}
};

drake::trajectories::PiecewisePolynomial<double> generateSpline(
    const SplineInformation& spline_information);

drake::trajectories::PiecewisePolynomial<double> nWaypointCubicSpline(
    const std::vector<double>& segment_times, double x0, double xd0, double xf,
    double xdf, const Eigen::Ref<const Eigen::VectorXd>& xi);
