#pragma once

#include <cstddef>

#include <Eigen/Core>

#include "drake/drakeOptimization_export.h"
#include "drake/solvers/Optimization.h"
#include "drake/systems/trajectories/PiecewisePolynomial.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

namespace drake {
namespace solvers {

/**
 * DirectTrajectoryOptimization An abstract class for direct method approaches
 * to trajectory optimization.
 *
 * This class assumes that there are a fixed number (N) time steps/samples, and
 * that the trajectory is discreteized into timesteps h (N-1 of these), state x
 * (N of these), and control input u (N of these).
 *
 * To maintain nominal sparsity in the optimization programs, this
 * implementation assumes that all constraints and costs are
 * time-invariant.
 */
class DRAKEOPTIMIZATION_EXPORT DirectTrajectoryOptimization {
 public:
  DirectTrajectoryOptimization(const int num_inputs, const int num_states,
                               const size_t num_time_samples,
                               const int trajectory_time_lower_bound,
                               const int trajectory_time_upper_bound);
  // TODO(lucy-tri) add param: time steps constant or independent.

  /**
   *  Evaluate the initial trajectories at the sampled times and construct the
   *  nominal initial vectors.
   */
  void GetInitialVars(int t_init_in,
                      const PiecewisePolynomial<double>& traj_init_u,
                      const PiecewisePolynomial<double>& traj_init_x);

  void AddStateConstraint(const Constraint& constraint, const int time_index);

  /** 
   * Solve the nonlinear program and return the resulting trajectory.
   * @p t_init Initial timespan for solution. 
   * @p traj_init_u Initial guess for trajectory for control input.
   * @p traj_init_x Initial guess for trajectory for state input.
   */
  SolutionResult SolveTraj(int t_init_in,
                           const PiecewisePolynomial<double>& traj_init_u,
                           const PiecewisePolynomial<double>& traj_init_x);
 private:
  const int num_inputs_;
  const int num_states_;
  const int num_vars_;
  const int N;  // Number of time samples

  OptimizationProblem opt_problem_;
  DecisionVariableView h_vars_;
  DecisionVariableView u_vars_;
  DecisionVariableView x_vars_;
};

}  // solvers
}  // drake
