#pragma once

#include <memory>

#include "drake/drakeTrajectoryOptimization_export.h"
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
class DRAKETRAJECTORYOPTIMIZATION_EXPORT DircolTrajectoryOptimization :
      public DirectTrajectoryOptimization {
 public:
  DircolTrajectoryOptimization(int num_inputs, int num_states,
                               int num_time_samples,
                               double trajectory_time_lower_bound,
                               double trajectory_time_upper_bound);

  /**
   * Adds a dynamic constraint to be applied to each pair of
   * states/inputs.
   */
  template <typename ConstraintT>
  void AddDynamicConstraint(
      const std::shared_ptr<ConstraintT>& constraint) {
    DRAKE_ASSERT(static_cast<int>(constraint->num_constraints()) ==
                 num_states());

    // For N-1 timesteps, add a constraint which depends on the knot
    // value along with the state and input vectors at that knot and the
    // next.
    for (int i = 0; i < N() - 1; i++) {
      opt_problem()->AddConstraint(
          constraint,
          {h_vars()(i), x_vars().segment(i * num_states(), num_states() * 2),
                u_vars().segment(i * num_inputs(), num_inputs() * 2)});
    }
  }

  /**
   * Adds an integrated cost to all time steps.
   *
   * @param constraint A constraint which expects a timestep, state,
   * and input as the elements of x when Eval is invoked.
   */
  void AddRunningCost(std::shared_ptr<Constraint> constraint);

  /**
   * Adds an integrated cost to all time steps.
   *
   * @param f A callable which meets the requirments of
   * OptimizationProblem::AddCost().
   */
  template <typename F>
  typename std::enable_if<
      !std::is_convertible<F, std::shared_ptr<Constraint>>::value,
      std::shared_ptr<Constraint>>::type
  AddRunningCost(F&& f) {
    auto c = OptimizationProblem::MakeCost(std::forward<F>(f));
    AddRunningCost(c);
    return c;
  }

  // TODO(sam.creasey) The MATLAB implementation of
  // DircolTrajectoryOptimization overrides the parent's
  // reconstructStateTrajectory to provide its own interpolation.
};

}  // solvers
}  // drake
