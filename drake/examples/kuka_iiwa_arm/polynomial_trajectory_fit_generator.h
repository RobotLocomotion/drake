#pragma once

#include <iostream>

#include "drake/drakeKukaIiwaArm_export.h"
#include "drake/common/polynomial.h"
#include "drake/systems/trajectories/PiecewisePolynomial.h"
#include "drake/systems/vector.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {

/// Generates a PiecewisePolynomial corresponding to a multi-dimensional
/// trajectory (specified as knot points and the corresponding segment_times.
/// The dimensionality of the polynomial is the same as that of the trajectory,
/// In the current implementation the polynomial order is at most one.
/// The number of @p segment_times must equal the number of columns on the
/// provided @joint_trajectories.
DRAKEKUKAIIWAARM_EXPORT
PiecewisePolynomial<double> PolynomialTrajectoryFitGenerator(
    const MatrixXd& joint_trajectories,
    const std::vector<double>& segment_times);

}  // namespace kuka_iia_arm
}  // namespace examples
}  // namespace drake
