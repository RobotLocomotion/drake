#pragma once

#include <Eigen/Core>

#include "drake/drakeTrajectoryOptimization_export.h"
#include "drake/solvers/Optimization.h"
#include "drake/systems/trajectories/PiecewisePolynomial.h"

namespace drake {
namespace solvers {

/**
 * DirectTrajectoryOptimization is a class for direct method approaches
 * to trajectory optimization.
 *
 * This class assumes that there are a fixed number (N) time steps/samples, and
 * that the trajectory is discreteized into timesteps h (N-1 of these), state x
 * (N of these), and control input u (N of these).
 *
 * To maintain nominal sparsity in the optimization programs, this
 * implementation assumes that all constraints and costs are
 * time-invariant.
 *
 * TODO(Lucy-tri) This class is a WIP.
 */
class DRAKETRAJECTORYOPTIMIZATION_EXPORT DirectTrajectoryOptimization {
 public:
  /**
   * @p trajectory_time_lower_bound Bound on total time for trajectory.
   * @p trajectory_time_upper_bound Bound on total time for trajectory.
   */
  DirectTrajectoryOptimization(int num_inputs, int num_states,
                               int num_time_samples,
                               double trajectory_time_lower_bound,
                               double trajectory_time_upper_bound);
  // TODO(Lucy-tri) add param: time steps constant or independent.

  /**
   * Solve the nonlinear program and return the resulting trajectory.
   *
   * @p t_init The final time of the solution.
   *
   * @p traj_init_u Initial guess for trajectory for control input. The number
   * of rows for each segment in @p traj_init_u must be equal to num_inputs
   * (the first param of the constructor).
   *
   * @p traj_init_x Initial guess for trajectory for state input. The number
   * of rows for each segment in @p traj_init_x must be equal to num_states
   * (the second param of the constructor).
   */
  SolutionResult SolveTraj(double t_init,
                           const PiecewisePolynomial<double>& traj_init_u,
                           const PiecewisePolynomial<double>& traj_init_x);
  // TODO(Lucy-tri) If t_init has any relationship to
  // trajectory_time_{lower,upper}_bound, then add doc and asserts.

  // Disable copy and assign.
  DirectTrajectoryOptimization(const DirectTrajectoryOptimization&) = delete;
  DirectTrajectoryOptimization& operator=(const DirectTrajectoryOptimization&) =
      delete;

 private:
  /**
   * Evaluate the initial trajectories at the sampled times and construct the
   * nominal initial vectors.
   *
   * @p t_init The final time of the solution.
   *
   * @p traj_init_u Initial guess for trajectory for control input. The number
   * of rows for each segment in @p traj_init_u must be equal to num_inputs
   * (the first param of the constructor).
   *
   * @p traj_init_x Initial guess for trajectory for state input. The number
   * of rows for each segment in @p traj_init_x must be equal to num_states
   * (the second param of the constructor).
   */
  void GetInitialVars(double t_init_in,
                      const PiecewisePolynomial<double>& traj_init_u,
                      const PiecewisePolynomial<double>& traj_init_x);

  const int num_inputs_;
  const int num_states_;
  const int N_;  // Number of time samples

  OptimizationProblem opt_problem_;
  DecisionVariableView h_vars_;
  DecisionVariableView u_vars_;
  DecisionVariableView x_vars_;
};

}  // solvers
}  // drake
