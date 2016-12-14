#pragma once

#include <string>
#include <vector>

#include "drake/common/eigen_types.h"
#include "drake/common/trajectories/piecewise_polynomial_trajectory.h"
#include "drake/multibody/rigid_body_ik.h"
#include "drake/multibody/rigid_body_tree.h"

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {

/**
 * Verify that @p tree matches assumptions about joint indices.
 * Aborts if the tree isn't as expected.
 */
void VerifyIiwaTree(const RigidBodyTree<double>& tree);

/// This method generates a trajectory plan for a robot. It assumes that the
/// robot is fixed in the world at a position specified by
/// @p robot_base_position, and @p robot_base_orientation. The robot itself
/// is defined by a URDF file @p robot_urdf_file.
std::unique_ptr<PiecewisePolynomialTrajectory> PositionViaPointCartesianPlanner(
    Eigen::Vector3d robot_base_position, Eigen::Vector3d robot_base_orientation,
    std::string robot_urdf_file,
    std::vector<Eigen::Vector3d> object_position_list,
    std::vector<double> time_stamps);

}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake
