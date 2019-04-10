#include "drake/systems/trajectory_optimization/multiple_shooting.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>

#include "drake/common/symbolic.h"
#include "drake/solvers/ipopt_solver.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

namespace drake {
namespace systems {
namespace trajectory_optimization {

using trajectories::PiecewisePolynomial;
using symbolic::Expression;
using internal::SequentialExpressionManager;

// For readability of long lines, these single-letter variables names are
// sometimes used:
// N number of timesteps/samples
// h timesteps (there are N-1 of these)
// x state
// u control input

MultipleShooting::MultipleShooting(int num_inputs, int num_states,
                                   int num_time_samples, double fixed_timestep)
    : MultipleShooting(num_inputs, num_states, num_time_samples,
                       false /* timesteps_are_decision_variables */,
                       fixed_timestep, fixed_timestep) {}

MultipleShooting::MultipleShooting(int num_inputs, int num_states,
                                   int num_time_samples,
                                   double minimum_timestep,
                                   double maximum_timestep)
    : MultipleShooting(num_inputs, num_states, num_time_samples,
                       true /* timesteps_are_decision_variables */,
                       minimum_timestep, maximum_timestep) {}

MultipleShooting::MultipleShooting(int num_inputs, int num_states,
                                   int num_time_samples,
                                   bool timesteps_are_decision_variables,
                                   double minimum_timestep,
                                   double maximum_timestep)
    : MathematicalProgram(),
      num_inputs_(num_inputs),
      num_states_(num_states),
      N_(num_time_samples),
      timesteps_are_decision_variables_(timesteps_are_decision_variables),
      fixed_timestep_(minimum_timestep),
      x_vars_(NewContinuousVariables(num_states_ * N_, "x")),
      u_vars_(NewContinuousVariables(num_inputs_ * N_, "u")),
      sequential_expression_manager_(N_) {
  placeholder_x_vars_ =
      sequential_expression_manager_.RegisterSequentialExpressions(
          Eigen::Map<solvers::MatrixXDecisionVariable>(x_vars_.data(),
                                                       num_states_, N_)
              .cast<symbolic::Expression>(),
          "x");
  placeholder_u_vars_ =
      sequential_expression_manager_.RegisterSequentialExpressions(
          Eigen::Map<solvers::MatrixXDecisionVariable>(u_vars_.data(),
                                                       num_inputs_, N_)
              .cast<symbolic::Expression>(),
          "u");
  DRAKE_DEMAND(num_time_samples > 1);
  DRAKE_DEMAND(num_states_ > 0);
  DRAKE_DEMAND(num_inputs_ >= 0);
  if (timesteps_are_decision_variables_) {
    h_vars_ = NewContinuousVariables(N_ - 1, "h");
    DRAKE_DEMAND(minimum_timestep >
                 0);  // == 0 tends to cause numerical issues.
    DRAKE_DEMAND(maximum_timestep >= minimum_timestep &&
                 std::isfinite(maximum_timestep));

    AddBoundingBoxConstraint(minimum_timestep, maximum_timestep, h_vars_);
    RowVectorX<Expression> t_expressions(N_);
    t_expressions(0) = 0;
    for (int i = 1; i < N_; ++i) {
      t_expressions(i) = t_expressions(i - 1) + h_vars_(i - 1);
    }
    placeholder_t_var_(0) =
        sequential_expression_manager_.RegisterSequentialExpressions(
            t_expressions, "t")(0);
  } else {
    h_vars_ = solvers::VectorXDecisionVariable(0);
    DRAKE_DEMAND(fixed_timestep_ > 0);
    placeholder_t_var_(0) =
        sequential_expression_manager_.RegisterSequentialExpressions(
            RowVectorX<Expression>::LinSpaced(N_, 0,
                                              (N_ - 1) * fixed_timestep_),
            "t")(0);
  }
}

solvers::VectorXDecisionVariable MultipleShooting::NewSequentialVariable(
    int rows, const std::string& name) {
  return sequential_expression_manager_.RegisterSequentialExpressions(
      NewContinuousVariables(rows, N_, name).cast<symbolic::Expression>(),
      name);
}

solvers::VectorXDecisionVariable MultipleShooting::GetSequentialVariableAtIndex(
    const std::string& name, int index) const {
  return symbolic::GetVariableVector(
      sequential_expression_manager_.GetSequentialExpressionsByName(name,
                                                                    index));
}

void MultipleShooting::AddTimeIntervalBounds(double lower_bound,
                                             double upper_bound) {
  DRAKE_THROW_UNLESS(timesteps_are_decision_variables_);
  AddBoundingBoxConstraint(lower_bound, upper_bound, h_vars_);
}

void MultipleShooting::AddEqualTimeIntervalsConstraints() {
  DRAKE_THROW_UNLESS(timesteps_are_decision_variables_);
  for (int i = 1; i < N_ - 1; i++) {
    AddLinearConstraint(h_vars_(i - 1) == h_vars_(i));
  }
}

void MultipleShooting::AddDurationBounds(double lower_bound,
                                         double upper_bound) {
  DRAKE_THROW_UNLESS(timesteps_are_decision_variables_);
  AddLinearConstraint(VectorXd::Ones(h_vars_.size()), lower_bound, upper_bound,
                      h_vars_);
}

solvers::Binding<solvers::VisualizationCallback>
MultipleShooting::AddInputTrajectoryCallback(
    const MultipleShooting::TrajectoryCallback& callback) {
  return AddVisualizationCallback(
      [this, callback](const Eigen::Ref<const Eigen::VectorXd>& x) {
        const Eigen::VectorXd times = GetSampleTimes(x.head(h_vars_.size()));
        const Eigen::Map<const Eigen::MatrixXd> inputs(
            x.data() + h_vars_.size(), num_inputs_, N_);
        callback(times, inputs);
      },
      {h_vars_, u_vars_});
}

solvers::Binding<solvers::VisualizationCallback>
MultipleShooting::AddStateTrajectoryCallback(
    const MultipleShooting::TrajectoryCallback& callback) {
  return AddVisualizationCallback(
      [this, callback](const Eigen::Ref<const Eigen::VectorXd>& x) {
        const Eigen::VectorXd times = GetSampleTimes(x.head(h_vars_.size()));
        const Eigen::Map<const Eigen::MatrixXd> states(
            x.data() + h_vars_.size(), num_states_, N_);
        callback(times, states);
      },
      {h_vars_, x_vars_});
}

void MultipleShooting::SetInitialTrajectory(
    const PiecewisePolynomial<double>& traj_init_u,
    const PiecewisePolynomial<double>& traj_init_x) {
  double start_time = 0;
  double h = fixed_timestep_;
  if (timesteps_are_decision_variables_) {
    double end_time = fixed_timestep_ * N_;
    DRAKE_THROW_UNLESS(!traj_init_u.empty() || !traj_init_x.empty());
    if (!traj_init_u.empty()) {
      start_time = traj_init_u.start_time();
      end_time = traj_init_u.end_time();
      if (!traj_init_x.empty()) {
        // Note: Consider adding a tolerance here if warranted.
        DRAKE_THROW_UNLESS(start_time == traj_init_x.start_time());
        DRAKE_THROW_UNLESS(end_time == traj_init_x.end_time());
      }
    } else {
      start_time = traj_init_x.start_time();
      end_time = traj_init_x.end_time();
    }
    DRAKE_DEMAND(start_time <= end_time);
    h = (end_time - start_time) / (N_ - 1);
    SetInitialGuess(h_vars_, VectorXd::Constant(h_vars_.size(), h));
  }

  VectorXd guess_u(u_vars_.size());
  if (traj_init_u.empty()) {
    guess_u.fill(0.003);  // Start with some small number <= 0.01.
  } else {
    for (int i = 0; i < N_; ++i) {
      guess_u.segment(num_inputs_ * i, num_inputs_) =
          traj_init_u.value(start_time + i * h);
    }
  }
  SetInitialGuess(u_vars_, guess_u);

  VectorXd guess_x(x_vars_.size());
  if (traj_init_x.empty()) {
    guess_x.fill(0.003);  // Start with some small number <= 0.01.
    // TODO(Lucy-tri) Do what DirectTrajectoryOptimization.m does.
  } else {
    for (int i = 0; i < N_; ++i) {
      guess_x.segment(num_states_ * i, num_states_) =
          traj_init_x.value(start_time + i * h);
    }
  }
  SetInitialGuess(x_vars_, guess_x);
}

Eigen::VectorXd MultipleShooting::GetSampleTimes(
    const Eigen::Ref<const Eigen::VectorXd>& h_var_values) const {
  Eigen::VectorXd times(N_);

  if (timesteps_are_decision_variables_) {
    times[0] = 0.0;
    for (int i = 1; i < N_; i++) {
      times[i] = times[i - 1] + h_var_values(i - 1);
    }
  } else {
    for (int i = 0; i < N_; i++) {
      times[i] = i * fixed_timestep_;
    }
  }
  return times;
}

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
Eigen::MatrixXd MultipleShooting::GetInputSamples() const {
  Eigen::MatrixXd inputs(num_inputs_, N_);
  for (int i = 0; i < N_; i++) {
    inputs.col(i) = GetSolution(input(i));
  }
  return inputs;
}
#pragma GCC diagnostic pop

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
Eigen::MatrixXd MultipleShooting::GetStateSamples() const {
  Eigen::MatrixXd states(num_states_, N_);
  for (int i = 0; i < N_; i++) {
    states.col(i) = GetSolution(state(i));
  }
  return states;
}
#pragma GCC diagnostic pop

Eigen::MatrixXd MultipleShooting::GetInputSamples(
    const solvers::MathematicalProgramResult& result) const {
  Eigen::MatrixXd inputs(num_inputs_, N_);
  for (int i = 0; i < N_; i++) {
    inputs.col(i) = result.GetSolution(input(i));
  }
  return inputs;
}

Eigen::MatrixXd MultipleShooting::GetStateSamples(
    const solvers::MathematicalProgramResult& result) const {
  Eigen::MatrixXd states(num_states_, N_);
  for (int i = 0; i < N_; i++) {
    states.col(i) = result.GetSolution(state(i));
  }
  return states;
}

symbolic::Substitution
MultipleShooting::ConstructPlaceholderVariableSubstitution(
    int interval_index) const {
  return sequential_expression_manager_
      .ConstructPlaceholderVariableSubstitution(interval_index);
}

symbolic::Expression MultipleShooting::SubstitutePlaceholderVariables(
    const symbolic::Expression& e, int interval_index) const {
  return e.Substitute(ConstructPlaceholderVariableSubstitution(interval_index));
}

symbolic::Formula MultipleShooting::SubstitutePlaceholderVariables(
    const symbolic::Formula& f, int interval_index) const {
  return f.Substitute(ConstructPlaceholderVariableSubstitution(interval_index));
}

}  // namespace trajectory_optimization
}  // namespace systems
}  // namespace drake
