#pragma once

#include <memory>

#include "drake/drakePendulum_export.h"
#include "drake/examples/Pendulum/Pendulum.h"
#include "drake/solvers/trajectoryOptimization/dircol_trajectory_optimization.h"

namespace drake {
namespace examples {
namespace pendulum {

/**
 * Populate a trajectory optimization with the correct
 * costs/limits/constraints for a swing up trajectory.  @p x0 and @p
 * xG define the starting and final states of the desired trajectory.
 */
void DRAKEPENDULUM_EXPORT AddSwingUpTrajectoryParams(
    std::shared_ptr<Pendulum>,
    int num_time_samples,
    const Eigen::Vector2d& x0, const Eigen::Vector2d& xG,
    solvers::DircolTrajectoryOptimization*);

}  // pendulum
}  // examples
}  // drake
