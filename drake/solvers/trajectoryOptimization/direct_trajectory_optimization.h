#pragma once

#include <vector>

#include <Eigen/Core>

#include "drake/common/drake_assert.h"
#include "drake/drakeTrajectoryOptimization_export.h"
#include "drake/solvers/optimization.h"
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
   * @param trajectory_time_lower_bound Bound on total time for
   *        trajectory.
   * @param trajectory_time_upper_bound Bound on total time for
   *        trajectory.
   */
  DirectTrajectoryOptimization(int num_inputs, int num_states,
                               int num_time_samples,
                               double trajectory_time_lower_bound,
                               double trajectory_time_upper_bound);
  // TODO(Lucy-tri) add param: time steps constant or independent.

  /**
   * Add a constraint on the input at the specified time indices.
   *
   * @param constraint The constraint to be applied.
   * @param time_indices The (0 offset) time indices to apply the
   *        constraint.
   */
  template <typename ConstraintT>
  void AddInputConstraint(std::shared_ptr<ConstraintT> constraint,
                          const std::vector<int>& time_indices) {
    for (const int i : time_indices) {
      DRAKE_ASSERT(i < N_);
      opt_problem_.AddConstraint(
          constraint, {u_vars_.segment(i * num_inputs_, num_inputs_)});
    }
  }

  /**
   * Add a constraint on the state at the specified time indices.
   *
   * @param constraint The constraint to be applied.
   * @param time_indices The (0 offset) time indices to apply the
   *        constraint.
   */
  template <typename ConstraintT>
  void AddStateConstraint(std::shared_ptr<ConstraintT> constraint,
                          const std::vector<int>& time_indices) {
    for (const int i : time_indices) {
      DRAKE_ASSERT(i < N_);
      opt_problem_.AddConstraint(
          constraint, {x_vars_.segment(i * num_states_, num_states_)});
    }
  }

  /**
   * Solve the nonlinear program and return the resulting trajectory.
   *
   * @param t_init The final time of the solution.
   *
   * @param traj_init_u Initial guess for trajectory for control
   * input. The number of rows for each segment in @p traj_init_u must
   * be equal to num_inputs (the first param of the constructor).
   *
   * @param traj_init_x Initial guess for trajectory for state
   * input. The number of rows for each segment in @p traj_init_x must
   * be equal to num_states (the second param of the constructor).
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

  /**
   * Extract the result of the trajectory solution as a set of
   * discrete samples.  Output matrices contain one set of input/state
   * per column, and the number of columns is equal to the number of
   * time samples.  @p times will be populated with the times
   * corresponding to each column.
   *
   * @param[out] inputs
   */
  void GetResultSamples(Eigen::MatrixXd* inputs, Eigen::MatrixXd* states,
                        std::vector<double>* times) const;

  /**
   * Get the input trajectory as a PiecewisePolynomial
   */
  PiecewisePolynomial<double> ReconstructInputTrajectory() const;

  /**
   * Get the state trajectory as a PiecewisePolynomial
   */
  PiecewisePolynomial<double> ReconstructStateTrajectory() const;

 protected:
  /**
   * @return a vector containing the elapsed time at each knot point.
   */
  std::vector<double> GetTimeVector() const;

  /**
   * @return a vector containing the input values at each knot point.
   */
  std::vector<Eigen::MatrixXd> GetInputVector() const;

  /**
   * @return a vector containing the state values at each knot point.
   */
  std::vector<Eigen::MatrixXd> GetStateVector() const;

 private:
  /**
   * Evaluate the initial trajectories at the sampled times and construct the
   * nominal initial vectors.
   *
   * @param t_init The final time of the solution.
   *
   * @param traj_init_u Initial guess for trajectory for control
   * input. The number of rows for each segment in @p traj_init_u must
   * be equal to num_inputs (the first param of the constructor).
   *
   * @param traj_init_x Initial guess for trajectory for state
   * input. The number of rows for each segment in @p traj_init_x must
   * be equal to num_states (the second param of the constructor).
   */
  void GetInitialVars(double t_init_in,
                      const PiecewisePolynomial<double>& traj_init_u,
                      const PiecewisePolynomial<double>& traj_init_x);

  const int num_inputs_;
  const int num_states_;
  const int N_;  // Number of time samples

  OptimizationProblem opt_problem_;
  DecisionVariableView h_vars_;  // Time deltas between each
                                 // input/state sample.
  DecisionVariableView u_vars_;
  DecisionVariableView x_vars_;
};

}  // solvers
}  // drake
