#pragma once

#include <iostream>

#include "drake/common/polynomial.h"
#include "drake/systems/trajectories/PiecewisePolynomial.h"
#include "drake/systems/vector.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {

PiecewisePolynomial<double> PolynomialTrajectoryFitGenerator(
    const MatrixXd& joint_trajectories,
    const std::vector<double>& timestamps);

}  // namespace kuka_iia_arm
}  // namespace examples
}  // namespace drake
