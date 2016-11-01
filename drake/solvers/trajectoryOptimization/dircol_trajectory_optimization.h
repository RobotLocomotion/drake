#pragma once

#include <memory>

#include "drake/common/drake_export.h"
#include "drake/solvers/constraint.h"
#include "drake/solvers/trajectoryOptimization/direct_trajectory_optimization.h"

namespace drake {
namespace solvers {

/**
 * DircolTrajectoryOptimization implements a direct colocation
 * approach to trajectory solving.  The main differences from
 * DirectTrajectoryOptimization are the addition of running costs and
 * dynamic constraints.
 */
class DRAKE_EXPORT DircolTrajectoryOptimization
    : public DirectTrajectoryOptimization {
 public:
  DircolTrajectoryOptimization(int num_inputs, int num_states,
                               int num_time_samples,
                               double trajectory_time_lower_bound,
                               double trajectory_time_upper_bound);

  void AddDynamicConstraint(
      const std::shared_ptr<Constraint>& constraint) override;

  void AddRunningCost(std::shared_ptr<Constraint> constraint) override;

  // TODO(sam.creasey) The MATLAB implementation of
  // DircolTrajectoryOptimization overrides the parent's
  // reconstructStateTrajectory to provide its own interpolation.
};

}  // namespace solvers
}  // namespace drake
