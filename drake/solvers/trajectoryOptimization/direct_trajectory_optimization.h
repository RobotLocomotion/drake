#pragma once

#include <vector>

#include <Eigen/Core>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_export.h"
#include "drake/solvers/mathematical_program.h"
#include "drake/systems/trajectories/PiecewisePolynomial.h"
#include "drake/systems/trajectories/piecewise_polynomial_trajectory.h"

namespace drake {
namespace solvers {

/**
 * DirectTrajectoryOptimization is an abstract class for direct method
 * approaches to trajectory optimization.
 *
 * Subclasses must implement the two abstract methods:
 *  AddRunningCost()
 * and
 *  AddDynamicConstraint()
 *
 * This class assumes that there are a fixed number (N) time steps/samples, and
 * that the trajectory is discretized into timesteps h (N-1 of these), state x
 * (N of these), and control input u (N of these).
 *
 * To maintain nominal sparsity in the optimization programs, this
 * implementation assumes that all constraints and costs are
 * time-invariant.
 */
class DRAKE_EXPORT DirectTrajectoryOptimization {
 public:
  /**
   * Adds a dynamic constraint to be applied to each pair of
   * states/inputs.
   */
  virtual void AddDynamicConstraint(
      const std::shared_ptr<Constraint>& constraint) = 0;

  /**
   * Adds an integrated cost to all time steps.
   *
   * @param constraint A constraint which expects a timestep, state,
   * and input as the elements of x when Eval is invoked.
   */
  virtual void AddRunningCost(std::shared_ptr<Constraint> constraint) = 0;

  /**
   * Adds an integrated cost to all time steps.
   *
   * @param f A callable which meets the requirments of
   * MathematicalProgram::AddCost().
   */
  template <typename F>
  typename std::enable_if<
      !std::is_convertible<F, std::shared_ptr<Constraint>>::value,
      std::shared_ptr<Constraint>>::type
  AddRunningCostFunc(F&& f) {
    auto c = MathematicalProgram::MakeCost(std::forward<F>(f));
    AddRunningCost(c);
    return c;
  }

  /**
   * Add upper and lower bounds on the input values.  Calling this
   * function multiple times will add additional bounds on the input
   * rather than resetting any bounds from previous invocations.
   */
  void AddInputBounds(const Eigen::VectorXd& lower_bound,
                      const Eigen::VectorXd& upper_bound);

  /**
   * Add a constraint on the input at the specified time indices.
   *
   * @param constraint The constraint to be applied.
   *
   * @param time_indices Apply the constraints only at these time
   * indices (zero offset).
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
   *
   * @param time_indices Apply the constraints only at these time
   * indices (zero offset).
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
   * Add bounds on a set of time intervals, such that
   * lower_bound(i) <= h_vars_(interval_indices[i]) <= upper_bound(i)
   * where h_vars_[j] is the time interval between j'th and j+1'th sample
   * (starting
   * from 0'th sample).
   * @param lower_bound  A vector of lower bounds.
   * @param upper_bound  A vector of upper bounds.
   * @param interval_indices A vector of interval indices.
   */
  void AddTimeIntervalBounds(const Eigen::VectorXd& lower_bound,
                             const Eigen::VectorXd& upper_bound,
                             const std::vector<int>& interval_indices);

  /**
   * Add bounds on all time intervals, such that
   * lower_bound(i) <= h_vars_(i) <= upper_bound(i)
   * where h_vars_[i] is the time interval between i'th and i+1'th sample
   * (starting
   * from 0'th sample).
   * @param lower_bound  A vector of lower bounds.
   * @param upper_bound  A vector of upper bounds.
   */
  void AddTimeIntervalBounds(const Eigen::VectorXd& lower_bound,
                             const Eigen::VectorXd& upper_bound);

  /**
   * Add a cost to the initial state.
   */
  template <typename ConstraintT>
  void AddInitialCost(std::shared_ptr<ConstraintT> constraint) {
    opt_problem_.AddCost(constraint, {x_vars_.head(num_states_)});
  }

  /**
   * Add a cost to the initial state.
   *
   * @param f A callable which meets the requirments of
   * MathematicalProgram::AddCost().
  */
  template <typename F>
  typename std::enable_if<
      !std::is_convertible<F, std::shared_ptr<Constraint>>::value,
      std::shared_ptr<Constraint>>::type
  AddInitialCostFunc(F&& f) {
    auto c = MathematicalProgram::MakeCost(std::forward<F>(f));
    AddInitialCost(c);
    return c;
  }

  /**
   * Add a cost to the final state and total time.
   *
   * @param constraint A constraint which expects total time as the
   * first element of x when Eval is invoked, followed by the final
   * state (num_states additional elements).
   */
  void AddFinalCost(std::shared_ptr<Constraint> constraint);

  /**
   * Add a cost to the final state and total time.
   *
   * @param f A callable which meets the requirments of
   * MathematicalProgram::AddCost().
   */
  template <typename F>
  typename std::enable_if<
      !std::is_convertible<F, std::shared_ptr<Constraint>>::value,
      std::shared_ptr<Constraint>>::type
  AddFinalCostFunc(F&& f) {
    auto c = MathematicalProgram::MakeCost(std::forward<F>(f));
    AddFinalCost(c);
    return c;
  }

  /**
   * Solve the nonlinear program and return the resulting trajectory.
   *
   * @param timespan_init The initial guess for the timespan of
   * the resulting trajectory.
   *
   * @param traj_init_u Initial guess for trajectory for control
   * input. The number of rows for each segment in @p traj_init_u must
   * be equal to num_inputs (the first param of the constructor).
   *
   * @param traj_init_x Initial guess for trajectory for state
   * input. The number of rows for each segment in @p traj_init_x must
   * be equal to num_states (the second param of the constructor).
   */
  SolutionResult SolveTraj(double timespan_init,
                           const PiecewisePolynomial<double>& traj_init_u,
                           const PiecewisePolynomial<double>& traj_init_x);
  // TODO(Lucy-tri) If timespan_init has any relationship to
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
   */
  void GetResultSamples(Eigen::MatrixXd* inputs, Eigen::MatrixXd* states,
                        std::vector<double>* times) const;

  /**
   * Get the input trajectory as a PiecewisePolynomialTrajectory
   */
  PiecewisePolynomialTrajectory ReconstructInputTrajectory() const;

  /**
   * Get the state trajectory as a PiecewisePolynomialTrajectory
   */
  PiecewisePolynomialTrajectory ReconstructStateTrajectory() const;

 protected:
  /**
   * Construct a DirectTrajectoryOptimization object. The dimensions
   * of the trajectory are established at construction, though other
   * parameters (costs, bounds, constraints, etc) can be set before
   * calling SolveTraj.
   *
   * @param num_inputs Number of inputs at each sample point.
   * @param num_states Number of states at each sample point.
   * @param num_time_samples Number of time samples.
   * @param trajectory_time_lower_bound Bound on total time for
   *        trajectory.
   * @param trajectory_time_upper_bound Bound on total time for
   *        trajectory.
   */
  DirectTrajectoryOptimization(int num_inputs, int num_states,
                               int num_time_samples,
                               double trajectory_time_lower_bound,
                               double trajectory_time_upper_bound);
  // TODO(Lucy-tri) add param to indicate whether time steps are constant or
  // independent, and modify implementation to handle.

  /**
   * Returns a vector containing the elapsed time at each knot point.
   */
  std::vector<double> GetTimeVector() const;

  /**
   * Returns a vector containing the input values at each knot point.
   */
  std::vector<Eigen::MatrixXd> GetInputVector() const;

  /**
   * Returns a vector containing the state values at each knot point.
   */
  std::vector<Eigen::MatrixXd> GetStateVector() const;

  int num_inputs() const { return num_inputs_; }
  int num_states() const { return num_states_; }
  int N() const { return N_; }
  MathematicalProgram* opt_problem() { return &opt_problem_; }
  const DecisionVariableView& h_vars() const { return h_vars_; }
  const DecisionVariableView& u_vars() const { return u_vars_; }
  const DecisionVariableView& x_vars() const { return x_vars_; }

 private:
  /**
   * Evaluate the initial trajectories at the sampled times and construct the
   * nominal initial vectors.
   *
   * @param timespan_init The final time of the solution.
   *
   * @param traj_init_u Initial guess for trajectory for control
   * input. The number of rows for each segment in @p traj_init_u must
   * be equal to num_inputs (the first param of the constructor).
   *
   * @param traj_init_x Initial guess for trajectory for state
   * input. The number of rows for each segment in @p traj_init_x must
   * be equal to num_states (the second param of the constructor).
   */
  void GetInitialVars(double timespan_init_in,
                      const PiecewisePolynomial<double>& traj_init_u,
                      const PiecewisePolynomial<double>& traj_init_x);

  const int num_inputs_;
  const int num_states_;
  const int N_;  // Number of time samples

  MathematicalProgram opt_problem_;
  DecisionVariableView h_vars_;  // Time deltas between each
                                 // input/state sample.
  DecisionVariableView u_vars_;
  DecisionVariableView x_vars_;
};

}  // namespace solvers
}  // namespace drake
