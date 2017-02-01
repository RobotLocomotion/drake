#pragma once

#include <memory>
#include <string>

#include "drake/common/trajectories/piecewise_polynomial_trajectory.h"

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {

/**
 * Generates a demo PiecewisePolynomialTrajectory for the Kuka iiwa arm.
 * @param urdf_path File path to the urdf model file for the Kuka iiwa arm.
 * @return unique pointer to the resulting PiecewisePolynomialTrajectory.
 */
std::unique_ptr<PiecewisePolynomialTrajectory>
MakeKukaDemoTrajectory(const std::string& urdf_path);

}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake
