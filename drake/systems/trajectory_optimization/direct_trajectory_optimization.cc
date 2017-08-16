#include "drake/systems/trajectory_optimization/direct_trajectory_optimization.h"

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

// For readability of long lines, these single-letter variables names are
// sometimes used:
// N number of timesteps/samples
// h timesteps (there are N-1 of these)
// x state
// u control input

namespace {

solvers::VectorXDecisionVariable MakeNamedVariables(const std::string prefix,
                                                    int num) {
  solvers::VectorXDecisionVariable vars(num);
  for (int i = 0; i < num; i++)
    vars(i) = symbolic::Variable(prefix + std::to_string(i));
  return vars;
}
}

DirectTrajectoryOptimization::DirectTrajectoryOptimization(
    int num_inputs, int num_states, int num_time_samples, double fixed_timestep)
    : num_inputs_(num_inputs),
      num_states_(num_states),
      N_(num_time_samples),
      timesteps_are_decision_variables_(false),
      fixed_timestep_(fixed_timestep),
      h_vars_(solvers::VectorXDecisionVariable(0)),
      x_vars_(NewContinuousVariables(num_states_ * N_, "x")),
      u_vars_(NewContinuousVariables(num_inputs_ * N_, "u")),
      placeholder_t_var_(
          solvers::VectorDecisionVariable<1>(symbolic::Variable("t"))),
      placeholder_x_vars_(MakeNamedVariables("x", num_states_)),
      placeholder_u_vars_(MakeNamedVariables("u", num_inputs_)) {
  DRAKE_DEMAND(num_time_samples > 1);
  DRAKE_DEMAND(num_states_ > 0);
  DRAKE_DEMAND(num_inputs_ >= 0);
  DRAKE_DEMAND(fixed_timestep > 0);
}

DirectTrajectoryOptimization::DirectTrajectoryOptimization(
    int num_inputs, int num_states, int num_time_samples,
    double minimum_timestep, double maximum_timestep)
    : num_inputs_(num_inputs),
      num_states_(num_states),
      N_(num_time_samples),
      timesteps_are_decision_variables_(true),
      h_vars_(NewContinuousVariables(N_ - 1, "h")),
      x_vars_(NewContinuousVariables(num_states_ * N_, "x")),
      u_vars_(NewContinuousVariables(num_inputs_ * N_, "u")),
      placeholder_t_var_(
          solvers::VectorDecisionVariable<1>(symbolic::Variable("t"))),
      placeholder_x_vars_(MakeNamedVariables("x", num_states_)),
      placeholder_u_vars_(MakeNamedVariables("u", num_inputs_)) {
  DRAKE_DEMAND(num_time_samples > 1);
  DRAKE_DEMAND(num_states_ > 0);
  DRAKE_DEMAND(num_inputs_ >= 0);
  DRAKE_DEMAND(minimum_timestep > 0);  // == 0 tends to cause numerical issues.
  DRAKE_DEMAND(maximum_timestep >= minimum_timestep &&
               std::isfinite(maximum_timestep));

  AddBoundingBoxConstraint(minimum_timestep, maximum_timestep, h_vars_);
}

void DirectTrajectoryOptimization::AddTimeIntervalBounds(
    const Eigen::VectorXd& lower_bound, const Eigen::VectorXd& upper_bound) {
  DRAKE_THROW_UNLESS(timesteps_are_decision_variables_);
  AddBoundingBoxConstraint(lower_bound, upper_bound, h_vars_);
}

void DirectTrajectoryOptimization::AddTimeIntervalBounds(
    const Eigen::VectorXd& lower_bound, const Eigen::VectorXd& upper_bound,
    const std::vector<int>& interval_indices) {
  DRAKE_THROW_UNLESS(timesteps_are_decision_variables_);
  solvers::VectorXDecisionVariable h(interval_indices.size());
  for (int i = 0; i < static_cast<int>(interval_indices.size()); ++i) {
    h(i) = h_vars_(interval_indices[i]);
  }
  AddBoundingBoxConstraint(lower_bound, upper_bound, h);
}

void DirectTrajectoryOptimization::AddTimeIntervalBounds(double lower_bound,
                                                         double upper_bound) {
  DRAKE_THROW_UNLESS(timesteps_are_decision_variables_);
  AddBoundingBoxConstraint(lower_bound, upper_bound, h_vars_);
}

void DirectTrajectoryOptimization::AddEqualTimeIntervalsConstraints() {
  DRAKE_THROW_UNLESS(timesteps_are_decision_variables_);
  for (int i = 1; i < N_ - 1; i++) {
    AddLinearConstraint(h_vars_(i - 1) == h_vars_(i));
  }
}

void DirectTrajectoryOptimization::AddDurationBounds(double lower_bound,
                                                     double upper_bound) {
  DRAKE_THROW_UNLESS(timesteps_are_decision_variables_);
  AddLinearConstraint(VectorXd::Ones(h_vars_.size()), lower_bound, upper_bound,
                      h_vars_);
}

void DirectTrajectoryOptimization::SetInitialTrajectory(
    const PiecewisePolynomial<double>& traj_init_u,
    const PiecewisePolynomial<double>& traj_init_x) {
  double start_time = 0;
  double h = fixed_timestep_;
  if (timesteps_are_decision_variables_) {
    double end_time = fixed_timestep_ * N_;
    DRAKE_THROW_UNLESS(!traj_init_u.empty() || !traj_init_x.empty());
    if (!traj_init_u.empty()) {
      start_time = traj_init_u.getStartTime();
      end_time = traj_init_u.getEndTime();
      if (!traj_init_x.empty()) {
        // Note: Consider adding a tolerance here if warranted.
        DRAKE_THROW_UNLESS(start_time == traj_init_x.getStartTime());
        DRAKE_THROW_UNLESS(end_time == traj_init_x.getEndTime());
      }
    } else {
      start_time = traj_init_x.getStartTime();
      end_time = traj_init_x.getEndTime();
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

std::vector<double> DirectTrajectoryOptimization::GetTimeVector() const {
  std::vector<double> times(N_);

  if (timesteps_are_decision_variables_) {
    const auto h_values = GetSolution(h_vars_);
    times[0] = 0.0;
    for (int i = 1; i < N_; i++) {
      times[i] = times[i - 1] + h_values(i - 1);
    }
  } else {
    for (int i = 0; i < N_; i++) {
      times[i] = i * fixed_timestep_;
    }
  }
  return times;
}

std::vector<Eigen::MatrixXd> DirectTrajectoryOptimization::GetInputVector()
    const {
  std::vector<Eigen::MatrixXd> inputs;
  inputs.reserve(N_);

  const auto u_values = GetSolution(u_vars_);

  for (int i = 0; i < N_; i++) {
    inputs.push_back(u_values.segment(i * num_inputs_, num_inputs_));
  }
  return inputs;
}

std::vector<Eigen::MatrixXd> DirectTrajectoryOptimization::GetStateVector()
    const {
  std::vector<Eigen::MatrixXd> states;
  states.reserve(N_);

  const auto x_values = GetSolution(x_vars_);

  for (int i = 0; i < N_; i++) {
    states.push_back(x_values.segment(i * num_states_, num_states_));
  }
  return states;
}

symbolic::Substitution
DirectTrajectoryOptimization::ConstructPlaceholderVariableSubstitution(
    int interval_index) const {
  symbolic::Substitution sub;

  if (timesteps_are_decision_variables_) {
    // time(i) is the sum of h intervals 0...(i-1)
    const symbolic::Expression time =
        h_vars_.head(interval_index).cast<symbolic::Expression>().sum();
    sub.emplace(placeholder_t_var_(0), time);
  } else {
    sub.emplace(placeholder_t_var_(0), interval_index * fixed_timestep_);
  }

  for (int i = 0; i < num_states_; i++)
    sub.emplace(placeholder_x_vars_(i),
                x_vars_(interval_index * num_states_ + i));
  for (int i = 0; i < num_inputs_; i++)
    sub.emplace(placeholder_u_vars_(i),
                u_vars_(interval_index * num_inputs_ + i));
  return sub;
}

symbolic::Expression
DirectTrajectoryOptimization::SubstitutePlaceholderVariables(
    const symbolic::Expression& e, int interval_index) const {
  return e.Substitute(ConstructPlaceholderVariableSubstitution(interval_index));
}

symbolic::Formula DirectTrajectoryOptimization::SubstitutePlaceholderVariables(
    const symbolic::Formula& f, int interval_index) const {
  return f.Substitute(ConstructPlaceholderVariableSubstitution(interval_index));
}

void DirectTrajectoryOptimization::GetResultSamples(
    Eigen::MatrixXd* inputs, Eigen::MatrixXd* states,
    std::vector<double>* times_out) const {
  std::vector<double> times = GetTimeVector();
  times_out->swap(times);

  inputs->resize(num_inputs_, N_);
  inputs->fill(0);
  states->resize(num_states_, N_);
  states->fill(0);

  const auto& u_values = GetSolution(u_vars_);
  const auto& x_values = GetSolution(x_vars_);

  for (int i = 0; i < N_; i++) {
    inputs->col(i) = u_values.segment(i * num_inputs_, num_inputs_);
    states->col(i) = x_values.segment(i * num_states_, num_states_);
  }
}

PiecewisePolynomialTrajectory
DirectTrajectoryOptimization::ReconstructInputTrajectory() const {
  return PiecewisePolynomialTrajectory(
      PiecewisePolynomial<double>::FirstOrderHold(GetTimeVector(),
                                                  GetInputVector()));
}

PiecewisePolynomialTrajectory
DirectTrajectoryOptimization::ReconstructStateTrajectory() const {
  return PiecewisePolynomialTrajectory(
      PiecewisePolynomial<double>::FirstOrderHold(GetTimeVector(),
                                                  GetStateVector()));
}

}  // namespace systems
}  // namespace drake
