// This file is based on pendulum_swing_up.h.

#pragma once

#include <memory>

#include "drake/systems/trajectory_optimization/direct_collocation.h"

namespace drake {
namespace examples {
namespace acrobot {

/**
 * Populates a DircolTrajectoryOptimization with the correct
 * costs/limits/constraints for a swing up trajectory.  @p x0 and @p
 * xG define the starting and final states of the desired trajectory.
 * @p num_time_samples is the number of time samples used to create
 * the DircolTrajectoryOptimization (and is the number of samples
 * between @p x0 and @p xG).
 */
void AddSwingUpTrajectoryParams(
    const Eigen::Vector4d& x0, const Eigen::Vector4d& xG,
    systems::DircolTrajectoryOptimization*);

}  // namespace acrobot
}  // namespace examples
}  // namespace drake
