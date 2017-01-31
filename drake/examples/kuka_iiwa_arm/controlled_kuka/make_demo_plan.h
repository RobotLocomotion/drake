#pragma once

#include <memory>
#include <string>

#include "drake/common/trajectories/piecewise_polynomial_trajectory.h"

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {

std::unique_ptr<PiecewisePolynomialTrajectory>
MakePlan(const std::string& path);

}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake
