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
namespace trajectory_optimization {

/// MultipleShooting is an abstract class for trajectory optimization
/// that creates decision variables for inputs, states, and (optionally)
/// sample times along the trajectory, then provides a number of methods
/// for working with those decision variables.
///
/// Subclasses must implement the abstract methods:
///  DoAddRunningCost()
///  ReconstructInputTrajectory()
///  ReconstructStateTrajectory()
/// using all of the correct interpolation schemes for the specific
/// transcription method, and should add the constraints to impose the
/// %System% dynamics in their constructor.
///
/// This class assumes that there are a fixed number (N) time steps/samples, and
/// that the trajectory is discretized into timesteps h (N-1 of these), state x
/// (N of these), and control input u (N of these).
class MultipleShooting : public solvers::MathematicalProgram {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MultipleShooting)

  ~MultipleShooting() override {}

  /// Returns the decision variable associated with the timestep, h, at time
  /// index @p index.
  /// @throws std::runtime_error if timesteps are not declared as decision
  /// variables.
  const solvers::VectorDecisionVariable<1> timestep(int index) const {
    DRAKE_THROW_UNLESS(timesteps_are_decision_variables_);
    DRAKE_DEMAND(index >= 0 && index < N_ - 1);
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

  /// Adds bounds on all time intervals.
  /// @param lower_bound  A scalar double lower bound.
  /// @param upper_bound  A scalar double upper bound.
  /// @throws std::runtime_error if timesteps are not declared as decision
  /// variables.
  void AddTimeIntervalBounds(double lower_bound, double upper_bound);

  /// Adds constraints to enforce that all timesteps have equal duration.
  /// @throws std::runtime_error if timesteps are not declared as decision
  /// variables.
  void AddEqualTimeIntervalsConstraints();

  /// Adds a constraint on the total duration of the trajectory.
  /// @throws std::runtime_error if timesteps are not declared as decision
  /// variables.
  void AddDurationBounds(double lower_bound, double upper_bound);

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
  ///    using MultipleShooting::AddFinalCost;
  /// to "unhide" this method.
  void AddFinalCost(const Eigen::Ref<const MatrixX<symbolic::Expression>>& e) {
    DRAKE_DEMAND(e.rows() == 1 && e.cols() == 1);
    AddFinalCost(e(0, 0));
  }

  /// Set the initial guess for the trajectory decision variables.
  ///
  /// @param traj_init_u Initial guess for trajectory for control
  /// input. The number of rows for each segment in @p traj_init_u must
  /// be equal to num_inputs (the first param of the constructor). If
  /// empty, then a default small non-zero initial value is used instead.
  ///
  /// @param traj_init_x Initial guess for trajectory for state
  /// input. The number of rows for each segment in @p traj_init_x must
  /// be equal to num_states (the second param of the constructor). If
  /// empty, then a default small non-zero initial value is used instead.
  ///
  /// If time steps are decision variables, then the initial guess for
  /// the time steps are evenly distributed to match the duration of the
  /// @p traj_init_u and @p traj_init_x. Throws std::runtime_error if
  /// @p traj_init_u and @p traj_init_x are both empty, or if
  /// @p traj_init_u and @p traj_init_x are both non-empty, and have
  /// different start and end times.
  // TODO(russt): Consider taking the actual breakpoints from
  // traj_init_{u,x} iff they match the number of sample times.
  void SetInitialTrajectory(const PiecewisePolynomial<double>& traj_init_u,
                            const PiecewisePolynomial<double>& traj_init_x);

  /// Returns a vector containing the elapsed time at each knot point at the
  /// solution.
  Eigen::VectorXd GetSampleTimes() const;

  /// Returns a matrix containing the input values (arranged in columns) at
  /// each knot point at the solution.
  Eigen::MatrixXd GetInputSamples() const;

  /// Returns a matrix containing the state values (arranged in columns) at
  /// each knot point at the solution.
  Eigen::MatrixXd GetStateSamples() const;

  /// Gets the input trajectory at the solution as a
  /// %PiecewisePolynomialTrajectory%.
  virtual PiecewisePolynomialTrajectory ReconstructInputTrajectory() const = 0;

  /// Gets the state trajectory at the solution as a
  /// %PiecewisePolynomialTrajectory%.
  virtual PiecewisePolynomialTrajectory ReconstructStateTrajectory() const = 0;

  double fixed_timestep() const {
    DRAKE_THROW_UNLESS(!timesteps_are_decision_variables_);
    return fixed_timestep_;
  }

 protected:
  /// Constructs a MultipleShooting instance with fixed sample times.
  ///
  /// @param num_inputs Number of inputs at each sample point.
  /// @param num_states Number of states at each sample point.
  /// @param num_time_samples Number of time samples.
  /// @param fixed_timestep The spacing between sample times.
  MultipleShooting(int num_inputs, int num_states, int num_time_samples,
                   double fixed_timestep);

  /// Constructs a MultipleShooting instance with sample times as decision
  /// variables.
  ///
  /// @param num_inputs Number of inputs at each sample point.
  /// @param num_states Number of states at each sample point.
  /// @param num_time_samples Number of time samples.
  /// @param minimum_timestep Minimum spacing between sample times.
  /// @param maximum_timestep Maximum spacing between sample times.
  MultipleShooting(int num_inputs, int num_states, int num_time_samples,
                   double minimum_timestep, double maximum_timestep);

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

  bool timesteps_are_decision_variables() const {
    return timesteps_are_decision_variables_;
  }

  const solvers::VectorXDecisionVariable& h_vars() const { return h_vars_; }

  const solvers::VectorXDecisionVariable& u_vars() const { return u_vars_; }

  const solvers::VectorXDecisionVariable& x_vars() const { return x_vars_; }

 private:
  virtual void DoAddRunningCost(const symbolic::Expression& g) = 0;

  // Helper method that performs the work for SubstitutePlaceHolderVariables
  symbolic::Substitution ConstructPlaceholderVariableSubstitution(
      int interval_index) const;

  const int num_inputs_{};
  const int num_states_{};
  const int N_{};  // Number of time samples
  const bool timesteps_are_decision_variables_{false};
  const double fixed_timestep_{0.0};

  solvers::VectorXDecisionVariable h_vars_;  // Time deltas between each
  // input/state sample or the
  // empty vector (if timesteps
  // are fixed).
  const solvers::VectorXDecisionVariable x_vars_;
  const solvers::VectorXDecisionVariable u_vars_;

  // See description of the public time(), state(), and input() accessor methods
  // for details about the placeholder variables.
  const solvers::VectorDecisionVariable<1> placeholder_t_var_;
  const solvers::VectorXDecisionVariable placeholder_x_vars_;
  const solvers::VectorXDecisionVariable placeholder_u_vars_;
};

}  // namespace trajectory_optimization
}  // namespace systems
}  // namespace drake
