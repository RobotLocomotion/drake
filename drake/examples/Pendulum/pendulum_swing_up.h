#pragma once

#include <memory>

#include "drake/common/drake_export.h"
#include "drake/solvers/trajectoryOptimization/dircol_trajectory_optimization.h"

namespace drake {
namespace examples {
namespace pendulum {

/**
 * Populates a DircolTrajectoryOptimization with the correct
 * costs/limits/constraints for a swing up trajectory.  @p x0 and @p
 * xG define the starting and final states of the desired trajectory.
 * @p num_time_samples is the number of time samples used to create
 * the DircolTrajectoryOptimization (and is the number of samples
 * between @p x0 and @p xG).
 */
void DRAKE_EXPORT AddSwingUpTrajectoryParams(
    int num_time_samples,
    const Eigen::Vector2d& x0, const Eigen::Vector2d& xG,
    solvers::DircolTrajectoryOptimization*);

}  // namespace pendulum
}  // namespace examples
}  // namespace drake
