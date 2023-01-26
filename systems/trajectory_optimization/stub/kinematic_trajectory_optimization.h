#pragma once

#warning The include path drake/systems/trajectory_optimization/kinematic_trajectory_optimization.h is deprecated and will be removed on or after 2023-05-01; instead, use drake/planning/trajectory_optimization/kinematic_trajectory_optimization.h.

#include "drake/planning/trajectory_optimization/kinematic_trajectory_optimization.h"

namespace drake {
namespace systems {
namespace trajectory_optimization {

using KinematicTrajectoryOptimization DRAKE_DEPRECATED(
    "2023-05-01",
    "Please use "
    "drake::planning::trajectory_optimization::KinematicTrajectoryOptimization "
    "instead.") =
    drake::planning::trajectory_optimization::KinematicTrajectoryOptimization;

}  // namespace trajectory_optimization
}  // namespace systems
}  // namespace drake
