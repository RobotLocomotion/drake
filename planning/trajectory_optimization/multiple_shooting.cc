#include "drake/planning/trajectory_optimization/multiple_shooting.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>

#include "drake/solvers/ipopt_solver.h"
#include "drake/solvers/solve.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

namespace drake {
namespace planning {
namespace trajectory_optimization {

using internal::SequentialExpressionManager;
using symbolic::Expression;
using symbolic::Formula;
using symbolic::MakeVectorContinuousVariable;
using symbolic::Substitution;
using symbolic::Variable;
using trajectories::PiecewisePolynomial;

// For readability of long lines, these single-letter variables names are
// sometimes used:
// N number of time steps (aka samples)
// h time steps (there are N-1 of these)
// x state
// u control input

MultipleShooting::~MultipleShooting() {}

MultipleShooting::MultipleShooting(int num_inputs, int num_states,
                                   int num_time_samples, double fixed_time_step,
                                   solvers::MathematicalProgram* prog)
    : MultipleShooting(num_inputs, num_states, num_time_samples,
                       false /* time_steps_are_decision_variables */,
                       fixed_time_step, fixed_time_step, prog) {}

MultipleShooting::MultipleShooting(
    const solvers::VectorXDecisionVariable& input,
    const solvers::VectorXDecisionVariable& state, int num_time_samples,
    double fixed_time_step, solvers::MathematicalProgram* prog)
    : MultipleShooting(input, state, num_time_samples, std::nullopt,
                       fixed_time_step, fixed_time_step, prog) {}

MultipleShooting::MultipleShooting(int num_inputs, int num_states,
                                   int num_time_samples,
                                   double minimum_time_step,
                                   double maximum_time_step,
                                   solvers::MathematicalProgram* prog)
    : MultipleShooting(num_inputs, num_states, num_time_samples,
                       true /* time_steps_are_decision_variables */,
                       minimum_time_step, maximum_time_step, prog) {}

MultipleShooting::MultipleShooting(
    const solvers::VectorXDecisionVariable& input,
    const solvers::VectorXDecisionVariable& state,
    const solvers::DecisionVariable& time, int num_time_samples,
    double minimum_time_step, double maximum_time_step,
    solvers::MathematicalProgram* prog)
    : MultipleShooting(input, state, num_time_samples, time, minimum_time_step,
                       maximum_time_step, prog) {}

MultipleShooting::MultipleShooting(int num_inputs, int num_states,
                                   int num_time_samples,
                                   bool time_steps_are_decision_variables,
                                   double minimum_time_step,
                                   double maximum_time_step,
                                   solvers::MathematicalProgram* prog)
    : MultipleShooting(
          MakeVectorContinuousVariable(num_inputs, "u"),
          MakeVectorContinuousVariable(num_states, "x"), num_time_samples,
          time_steps_are_decision_variables
              ? std::optional<
                    solvers::DecisionVariable>{solvers::DecisionVariable{"t"}}
              : std::nullopt,
          minimum_time_step, maximum_time_step, prog) {}

MultipleShooting::MultipleShooting(
    const solvers::VectorXDecisionVariable& input,
    const solvers::VectorXDecisionVariable& state, int num_time_samples,
    const std::optional<solvers::DecisionVariable>& time_var,
    double minimum_time_step, double maximum_time_step,
    solvers::MathematicalProgram* prog)
    : owned_prog_(prog ? nullptr
                       : std::make_unique<solvers::MathematicalProgram>()),
      prog_(prog ? *prog : *owned_prog_),
      num_inputs_(input.size()),
      num_states_(state.size()),
      N_(num_time_samples),
      time_steps_are_decision_variables_(time_var),
      fixed_time_step_(minimum_time_step),
      x_vars_(prog_.NewContinuousVariables(num_states_ * N_, "x")),
      u_vars_(prog_.NewContinuousVariables(num_inputs_ * N_, "u")),
      placeholder_x_vars_(state),
      placeholder_u_vars_(input),
      sequential_expression_manager_(N_) {
  sequential_expression_manager_.RegisterSequentialExpressions(
      state,
      Eigen::Map<solvers::MatrixXDecisionVariable>(x_vars_.data(), num_states_,
                                                   N_)
          .cast<Expression>(),
      "x");
  sequential_expression_manager_.RegisterSequentialExpressions(
      input,
      Eigen::Map<solvers::MatrixXDecisionVariable>(u_vars_.data(), num_inputs_,
                                                   N_)
          .cast<Expression>(),
      "u");
  DRAKE_DEMAND(num_time_samples > 1);
  DRAKE_DEMAND(num_states_ > 0);
  DRAKE_DEMAND(num_inputs_ >= 0);
  if (time_steps_are_decision_variables_) {
    h_vars_ = prog_.NewContinuousVariables(N_ - 1, "h");
    DRAKE_DEMAND(minimum_time_step >
                 0);  // == 0 tends to cause numerical issues.
    DRAKE_DEMAND(maximum_time_step >= minimum_time_step &&
                 std::isfinite(maximum_time_step));

    prog_.AddBoundingBoxConstraint(minimum_time_step, maximum_time_step,
                                   h_vars_);
    RowVectorX<Expression> t_expressions(N_);
    t_expressions(0) = 0;
    for (int i = 1; i < N_; ++i) {
      t_expressions(i) = t_expressions(i - 1) + h_vars_(i - 1);
    }
    placeholder_t_var_(0) = *time_var;
    sequential_expression_manager_.RegisterSequentialExpressions(
        placeholder_t_var_, t_expressions, "t");
  } else {
    h_vars_ = solvers::VectorXDecisionVariable(0);
    DRAKE_DEMAND(fixed_time_step_ > 0);
    placeholder_t_var_(0) =
        sequential_expression_manager_.RegisterSequentialExpressions(
            RowVectorX<Expression>::LinSpaced(N_, 0,
                                              (N_ - 1) * fixed_time_step_),
            "t")(0);
  }
}

solvers::VectorXDecisionVariable MultipleShooting::NewSequentialVariable(
    int rows, const std::string& name) {
  return sequential_expression_manager_.RegisterSequentialExpressions(
      prog_.NewContinuousVariables(rows, N_, name).cast<Expression>(), name);
}

solvers::VectorXDecisionVariable MultipleShooting::GetSequentialVariableAtIndex(
    const std::string& name, int index) const {
  return symbolic::GetVariableVector(
      sequential_expression_manager_.GetSequentialExpressionsByName(name,
                                                                    index));
}

std::vector<solvers::Binding<solvers::Constraint>>
MultipleShooting::AddConstraintToAllKnotPoints(
    const Eigen::Ref<const VectorX<symbolic::Formula>>& f) {
  std::vector<solvers::Binding<solvers::Constraint>> bindings;
  for (int i = 0; i < f.size(); ++i) {
    std::vector<solvers::Binding<solvers::Constraint>> b =
        AddConstraintToAllKnotPoints(f[i]);
    bindings.insert(bindings.end(), std::make_move_iterator(b.begin()),
                    std::make_move_iterator(b.end()));
  }
  return bindings;
}

solvers::Binding<solvers::BoundingBoxConstraint>
MultipleShooting::AddTimeIntervalBounds(double lower_bound,
                                        double upper_bound) {
  DRAKE_THROW_UNLESS(time_steps_are_decision_variables_);
  return prog_.AddBoundingBoxConstraint(lower_bound, upper_bound, h_vars_);
}

std::vector<solvers::Binding<solvers::LinearConstraint>>
MultipleShooting::AddEqualTimeIntervalsConstraints() {
  DRAKE_THROW_UNLESS(time_steps_are_decision_variables_);
  std::vector<solvers::Binding<solvers::LinearConstraint>> constraints;
  for (int i = 1; i < N_ - 1; i++) {
    constraints.push_back(
        prog_.AddLinearConstraint(h_vars_(i - 1) == h_vars_(i)));
  }
  return constraints;
}

solvers::Binding<solvers::LinearConstraint> MultipleShooting::AddDurationBounds(
    double lower_bound, double upper_bound) {
  DRAKE_THROW_UNLESS(time_steps_are_decision_variables_);
  return prog_.AddLinearConstraint(VectorXd::Ones(h_vars_.size()), lower_bound,
                                   upper_bound, h_vars_);
}

solvers::Binding<solvers::VisualizationCallback>
MultipleShooting::AddInputTrajectoryCallback(
    const MultipleShooting::TrajectoryCallback& callback) {
  return prog_.AddVisualizationCallback(
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
  return prog_.AddVisualizationCallback(
      [this, callback](const Eigen::Ref<const Eigen::VectorXd>& x) {
        const Eigen::VectorXd times = GetSampleTimes(x.head(h_vars_.size()));
        const Eigen::Map<const Eigen::MatrixXd> states(
            x.data() + h_vars_.size(), num_states_, N_);
        callback(times, states);
      },
      {h_vars_, x_vars_});
}

solvers::Binding<solvers::VisualizationCallback>
MultipleShooting::AddCompleteTrajectoryCallback(
    const MultipleShooting::CompleteTrajectoryCallback& callback,
    const std::vector<std::string>& names) {
  int num_extra_vars = 0;
  std::vector<int> size_extra_vars;
  for (size_t ii = 0; ii < names.size(); ii++) {
    int num_rows = sequential_expression_manager_.num_rows(names[ii]);
    num_extra_vars += num_rows;
    size_extra_vars.push_back(num_rows);
  }
  solvers::VectorXDecisionVariable extra_vars(num_extra_vars * N_);
  int row_offset = 0;
  for (size_t ii = 0; ii < names.size(); ii++) {
    extra_vars.middleRows(row_offset, size_extra_vars[ii] * N_) =
        GetSequentialVariable(names[ii]);
    row_offset += size_extra_vars[ii] * N_;
  }

  return prog_.AddVisualizationCallback(
      [this, callback,
       size_extra_vars](const Eigen::Ref<const Eigen::VectorXd>& x) {
        const Eigen::VectorXd times = GetSampleTimes(x.head(h_vars_.size()));
        const Eigen::Map<const Eigen::MatrixXd> states(
            x.data() + h_vars_.size(), num_states_, N_);
        const Eigen::Map<const Eigen::MatrixXd> inputs(
            x.data() + h_vars_.size() + x_vars_.size(), num_inputs_, N_);
        int data_offset = h_vars_.size() + u_vars_.size() + x_vars_.size();
        std::vector<Eigen::Ref<const Eigen::MatrixXd>> extras_vec;
        for (size_t ii = 0; ii < size_extra_vars.size(); ii++) {
          const Eigen::Map<const Eigen::MatrixXd> extras(
              x.data() + data_offset, size_extra_vars[ii], N_);
          data_offset += size_extra_vars[ii] * N_;
          extras_vec.push_back(extras);
        }
        callback(times, states, inputs, extras_vec);
      },
      {h_vars_, x_vars_, u_vars_, extra_vars});
}

void MultipleShooting::SetInitialTrajectory(
    const PiecewisePolynomial<double>& traj_init_u,
    const PiecewisePolynomial<double>& traj_init_x) {
  double start_time = 0;
  double h = fixed_time_step_;
  if (time_steps_are_decision_variables_) {
    double end_time = fixed_time_step_ * N_;
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
    prog_.SetInitialGuess(h_vars_, VectorXd::Constant(h_vars_.size(), h));
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
  prog_.SetInitialGuess(u_vars_, guess_u);

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
  prog_.SetInitialGuess(x_vars_, guess_x);
}

Eigen::VectorXd MultipleShooting::GetSampleTimes(
    const Eigen::Ref<const Eigen::VectorXd>& h_var_values) const {
  Eigen::VectorXd times(N_);

  if (time_steps_are_decision_variables_) {
    times[0] = 0.0;
    for (int i = 1; i < N_; i++) {
      times[i] = times[i - 1] + h_var_values(i - 1);
    }
  } else {
    for (int i = 0; i < N_; i++) {
      times[i] = i * fixed_time_step_;
    }
  }
  return times;
}

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

Eigen::MatrixXd MultipleShooting::GetSequentialVariableSamples(
    const solvers::MathematicalProgramResult& result,
    const std::string& name) const {
  const int num_sequential_variables =
      sequential_expression_manager_.num_rows(name);
  Eigen::MatrixXd sequential_variables(num_sequential_variables, N_);
  for (int i = 0; i < N_; i++) {
    sequential_variables.col(i) =
        result.GetSolution(GetSequentialVariableAtIndex(name, i));
  }
  return sequential_variables;
}

Substitution MultipleShooting::ConstructPlaceholderVariableSubstitution(
    int interval_index) const {
  return sequential_expression_manager_
      .ConstructPlaceholderVariableSubstitution(interval_index);
}

Expression MultipleShooting::SubstitutePlaceholderVariables(
    const Expression& e, int interval_index) const {
  return e.Substitute(ConstructPlaceholderVariableSubstitution(interval_index));
}

const solvers::VectorXDecisionVariable MultipleShooting::GetSequentialVariable(
    const std::string& name) const {
  const int rows = sequential_expression_manager_.num_rows(name);
  VectorX<Expression> sequential_variable(rows * N_);
  for (int i = 0; i < N_; i++) {
    sequential_variable.segment(i * rows, rows) =
        sequential_expression_manager_.GetSequentialExpressionsByName(name, i);
  }
  return symbolic::GetVariableVector(sequential_variable);
}

Formula MultipleShooting::SubstitutePlaceholderVariables(
    const Formula& f, int interval_index) const {
  return f.Substitute(ConstructPlaceholderVariableSubstitution(interval_index));
}

}  // namespace trajectory_optimization
}  // namespace planning
}  // namespace drake
