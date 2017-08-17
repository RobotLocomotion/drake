#pragma once

#include <memory>
#include <utility>
#include <vector>

#include <Eigen/Core>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/symbolic.h"
#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/common/trajectories/piecewise_polynomial_trajectory.h"
#include "drake/solvers/mathematical_program.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/system.h"

namespace drake {
namespace systems {

///
/// DirectTrajectoryOptimization is an abstract class for direct method
/// approaches to trajectory optimization.
///
/// Subclasses must implement the abstract method:
///  DoAddRunningCost()
/// and should add any dynamic constraints in their constructor.
///
/// This class assumes that there are a fixed number (N) time steps/samples, and
/// that the trajectory is discretized into timesteps h (N-1 of these), state x
/// (N of these), and control input u (N of these).
///
/// To maintain nominal sparsity in the optimization programs, this
/// implementation assumes that all constraints and costs are
/// time-invariant.
class DirectTrajectoryOptimization : public solvers::MathematicalProgram {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(DirectTrajectoryOptimization)

  ~DirectTrajectoryOptimization() override {}

  /// Returns the decision variable associated with the timestep, h, at time
  /// index @p index.
  const solvers::VectorDecisionVariable<1> timestep(int index) const {
    DRAKE_DEMAND(index >= 0 && index < N_);
    return h_vars_.segment<1>(index);
  }

  /// Returns a placeholder decision variable (not actually declared as a
  /// decision variable in the MathematicalProgram) associated with the time, t.
  /// This variable will be substituted for real decision variables at
  /// particular times in methods like AddRunningCost.  Passing this variable
  /// directly into objectives/constraints for the parent classes will result
  /// in an error.
  const solvers::VectorDecisionVariable<1>& time() const {
    return placeholder_t_var_;
  }

  /// Returns placeholder decision variables (not actually declared as decision
  /// variables in the MathematicalProgram) associated with the state, x, but
  /// with the time-index undetermined.  These variables will be substituted
  /// for real decision variables at particular times in methods like
  /// AddRunningCost.  Passing these variables directly into
  /// objectives/constraints for the parent classes will result in an error.
  const solvers::VectorXDecisionVariable& state() const {
    return placeholder_x_vars_;
  }

  /// Returns placeholder decision variables (not actually declared as decision
  /// variables in the MathematicalProgram) associated with the input, u, but
  /// with the time-index undetermined.  These variables will be substituted
  /// for real decision variables at particular times in methods like
  /// AddRunningCost.  Passing these variables directly into
  /// objectives/constraints for the parent classes will result in an error.
  const solvers::VectorXDecisionVariable& input() const {
    return placeholder_u_vars_;
  }

  /// Returns the decision variables associated with the state, x, at time
  /// index @p index.
  Eigen::VectorBlock<const solvers::VectorXDecisionVariable> state(
      int index) const {
    DRAKE_DEMAND(index >= 0 && index < N_);
    return x_vars_.segment(index * num_states_, num_states_);
  }

  /// Returns the decision variables associated with the state, x, at the
  /// initial time index.
  Eigen::VectorBlock<const solvers::VectorXDecisionVariable> initial_state()
      const {
    return state(0);
  }

  /// Returns the decision variables associated with the state, x, at the final
  /// time index.
  Eigen::VectorBlock<const solvers::VectorXDecisionVariable> final_state()
      const {
    return state(N_ - 1);
  }

  /// Returns the decision variables associated with the input, u, at time
  /// index @p index.
  Eigen::VectorBlock<const solvers::VectorXDecisionVariable> input(
      int index) const {
    DRAKE_DEMAND(index >= 0 && index < N_);
    return u_vars_.segment(index * num_inputs_, num_inputs_);
  }

  /// Adds an integrated cost to all time steps, of the form
  ///    @f[ cost = \int_0^T g(t,x,u) dt, @f]
  /// where any instances of time(), state(), and/or input() placeholder
  /// variables are substituted with the relevant variables for each current
  /// time index.  The particular integration scheme is determined by the
  /// derived class implementation.
  void AddRunningCost(const symbolic::Expression& g) { DoAddRunningCost(g); }

  /// Adds support for passing in a (scalar) matrix Expression, which is a
  /// common output of most symbolic linear algebra operations.
  template <typename Derived>
  void AddRunningCost(const Eigen::MatrixBase<Derived>& g) {
    DRAKE_DEMAND(g.rows() == 1 && g.cols() == 1);
    DoAddRunningCost(g(0, 0));
  }

  /// Adds a constraint to all knot points, where any instances of time(),
  /// state(), and/or input() placeholder variables are substituted with the
  /// relevant variables for each current time index.
  void AddConstraintToAllKnotPoints(const symbolic::Formula& f) {
    for (int i = 0; i < N_; i++) {
      // TODO(russt): update this to AddConstraint once MathematicalProgram
      // supports AddConstraint for Formulas.
      // For now, non-linear constraints can be added by users by simply adding
      // the constraint manually for each
      // time index in a loop.
      AddLinearConstraint(SubstitutePlaceholderVariables(f, i));
    }
  }

  // TODO(russt): Add additional cost/constraint wrappers that assign e.g.
  // non-symbolic costs (like QuadraticCost)
  // by taking in a list of vars that could contain placeholder vars, or that
  // assign costs/constraints to a set of
  // interval indices.

  /// Add bounds on a set of time intervals, such that
  /// lower_bound(i) <= h_vars_(interval_indices[i]) <= upper_bound(i)
  /// where h_vars_[j] is the time interval between j'th and j+1'th sample
  /// (starting
  /// from 0'th sample).
  /// @param lower_bound  A vector of lower bounds.
  /// @param upper_bound  A vector of upper bounds.
  /// @param interval_indices A vector of interval indices.
  void AddTimeIntervalBounds(const Eigen::VectorXd& lower_bound,
                             const Eigen::VectorXd& upper_bound,
                             const std::vector<int>& interval_indices);

  /// Add bounds on all time intervals, such that
  /// lower_bound(i) <= h_vars_(i) <= upper_bound(i)
  /// where h_vars_[i] is the time interval between i'th and i+1'th sample
  /// (starting
  /// from 0'th sample).
  /// @param lower_bound  A vector of lower bounds.
  /// @param upper_bound  A vector of upper bounds.
  void AddTimeIntervalBounds(const Eigen::VectorXd& lower_bound,
                             const Eigen::VectorXd& upper_bound);

  /// Add bounds on all time intervals, such that
  /// lower_bound <= h_vars_(i) <= upper_bound
  /// for all time intervals.
  /// @param lower_bound  A scalar double lower bound.
  /// @param upper_bound  A scalar double upper bound.
  void AddTimeIntervalBounds(double lower_bound, double upper_bound);

  /// Adds a cost to the final time, of the form
  ///    @f[ cost = e(t,x,u), @f]
  /// where any instances of time(), state(), and/or input() placeholder
  /// variables are substituted with the relevant variables for each current
  /// time index.
  void AddFinalCost(const symbolic::Expression& e) {
    AddCost(SubstitutePlaceholderVariables(e, N_ - 1));
  }

  /// Adds support for passing in a (scalar) matrix Expression, which is a
  /// common output of most symbolic linear algebra operations.
  /// Note: Derived classes will need to type
  ///    using DirectTrajectoryOptimization::AddFinalCost;
  /// to "unhide" this method.
  void AddFinalCost(const Eigen::Ref<const MatrixX<symbolic::Expression>>& e) {
    DRAKE_DEMAND(e.rows() == 1 && e.cols() == 1);
    AddFinalCost(e(0, 0));
  }

  /// Solve the nonlinear program and return the resulting trajectory.
  ///
  /// @param timespan_init The initial guess for the timespan of
  /// the resulting trajectory.
  ///
  /// @param traj_init_u Initial guess for trajectory for control
  /// input. The number of rows for each segment in @p traj_init_u must
  /// be equal to num_inputs (the first param of the constructor).
  ///
  /// @param traj_init_x Initial guess for trajectory for state
  /// input. The number of rows for each segment in @p traj_init_x must
  /// be equal to num_states (the second param of the constructor).
  solvers::SolutionResult SolveTraj(
      double timespan_init, const PiecewisePolynomial<double>& traj_init_u,
      const PiecewisePolynomial<double>& traj_init_x);
  // TODO(Lucy-tri) If timespan_init has any relationship to
  // trajectory_time_{lower,upper}_bound, then add doc and asserts.

  /// Extract the result of the trajectory solution as a set of
  /// discrete samples.  Output matrices contain one set of input/state
  /// per column, and the number of columns is equal to the number of
  /// time samples.  @p times will be populated with the times
  /// corresponding to each column.
  void GetResultSamples(Eigen::MatrixXd* inputs, Eigen::MatrixXd* states,
                        std::vector<double>* times) const;

  /// Get the input trajectory as a PiecewisePolynomialTrajectory
  PiecewisePolynomialTrajectory ReconstructInputTrajectory() const;

  /// Get the state trajectory as a PiecewisePolynomialTrajectory
  virtual PiecewisePolynomialTrajectory ReconstructStateTrajectory() const;

 protected:
  /// Construct a DirectTrajectoryOptimization object. The dimensions
  /// of the trajectory are established at construction, though other
  /// parameters (costs, bounds, constraints, etc) can be set before
  /// calling SolveTraj.
  ///
  /// @param num_inputs Number of inputs at each sample point.
  /// @param num_states Number of states at each sample point.
  /// @param num_time_samples Number of time samples.
  /// @param trajectory_time_lower_bound Bound on total time for
  ///        trajectory.
  /// @param trajectory_time_upper_bound Bound on total time for
  ///        trajectory.
  // TODO(russt): update comment above. Note that system/context are cloned
  // internally... must use accessors to make any changes to the system.
  DirectTrajectoryOptimization(int num_inputs, int num_states,
                               int num_time_samples,
                               double trajectory_time_lower_bound,
                               double trajectory_time_upper_bound);
  // TODO(Lucy-tri) add param to indicate whether time steps are constant or
  // independent, and modify implementation to handle.

  /// Returns a vector containing the elapsed time at each knot point.
  std::vector<double> GetTimeVector() const;

  /// Returns a vector containing the input values at each knot point.
  std::vector<Eigen::MatrixXd> GetInputVector() const;

  /// Returns a vector containing the state values at each knot point.
  std::vector<Eigen::MatrixXd> GetStateVector() const;

  /// Replaces e.g. placeholder_x_var_ with x_vars_ at time interval
  /// @p interval_index, for all placeholder variables.
  symbolic::Expression SubstitutePlaceholderVariables(
      const symbolic::Expression& e, int interval_index) const;

  /// Replaces e.g. placeholder_x_var_ with x_vars_ at time interval
  /// @p interval_index, for all placeholder variables.
  symbolic::Formula SubstitutePlaceholderVariables(const symbolic::Formula& f,
                                                   int interval_index) const;

  int num_inputs() const { return num_inputs_; }
  int num_states() const { return num_states_; }
  int N() const { return N_; }
  const solvers::VectorXDecisionVariable& h_vars() const { return h_vars_; }
  const solvers::VectorXDecisionVariable& u_vars() const { return u_vars_; }
  const solvers::VectorXDecisionVariable& x_vars() const { return x_vars_; }

 private:
  // Evaluate the initial trajectories at the sampled times and construct the
  // nominal initial vectors.
  //
  // @param timespan_init The final time of the solution.
  //
  // @param traj_init_u Initial guess for trajectory for control
  // input. The number of rows for each segment in @p traj_init_u must
  // be equal to num_inputs (the first param of the constructor).
  //
  // @param traj_init_x Initial guess for trajectory for state
  // input. The number of rows for each segment in @p traj_init_x must
  // be equal to num_states (the second param of the constructor).
  void GetInitialVars(double timespan_init_in,
                      const PiecewisePolynomial<double>& traj_init_u,
                      const PiecewisePolynomial<double>& traj_init_x);

  virtual void DoAddRunningCost(const symbolic::Expression& g) = 0;

  // Helper method that performs the work for SubstitutePlaceHolderVariables
  symbolic::Substitution ConstructPlaceholderVariableSubstitution(
      int interval_index) const;

  const int num_inputs_{};
  const int num_states_{};
  const int N_{};  // Number of time samples

  solvers::VectorXDecisionVariable h_vars_;  // Time deltas between each
                                             // input/state sample.
  solvers::VectorXDecisionVariable x_vars_;
  solvers::VectorXDecisionVariable u_vars_;

  // See description of the public time(), state(), and input() accessor methods
  // for details about the placeholder variables.
  solvers::VectorDecisionVariable<1> placeholder_t_var_;
  solvers::VectorXDecisionVariable placeholder_x_vars_;
  solvers::VectorXDecisionVariable placeholder_u_vars_;
};

}  // namespace systems
}  // namespace drake
