#include "drake/manipulation/planner/kinematic_trajectory_optimization.h"

#include <algorithm>
#include <string>

#include "drake/common/text_logging.h"
#include "drake/math/autodiff.h"
#include "drake/math/autodiff_gradient.h"
#include "drake/math/bspline_basis.h"
#include "drake/solvers/scs_solver.h"
#include "drake/solvers/snopt_solver.h"

using drake::math::BsplineBasis;
using drake::math::BsplineCurve;
using drake::math::autoDiffToValueMatrix;
using drake::math::initializeAutoDiff;
using drake::solvers::Constraint;
using drake::solvers::MathematicalProgram;
using drake::solvers::MatrixXDecisionVariable;
using drake::solvers::SnoptSolver;
using drake::solvers::SolutionResult;
using drake::solvers::VectorXDecisionVariable;
using drake::trajectories::PiecewisePolynomial;
using drake::trajectories::PiecewiseTrajectory;

namespace symbolic = drake::symbolic;
namespace trajectories = drake::trajectories;

namespace drake {
namespace manipulation {
namespace planner {
namespace {
VectorXDecisionVariable MakeNamedVariables(const std::string& prefix, int num) {
  VectorXDecisionVariable vars(num);
  for (int i = 0; i < num; i++)
    vars(i) = symbolic::Variable(prefix + std::to_string(i));
  return vars;
}

class PointConstraint : public Constraint {
 public:
  PointConstraint(std::shared_ptr<Constraint> wrapped_constraint,
                  const std::vector<double>& basis_function_values)
      : Constraint(
            wrapped_constraint->num_outputs(),
            basis_function_values.size() * wrapped_constraint->num_vars(),
            wrapped_constraint->lower_bound(),
            wrapped_constraint->upper_bound()),
        wrapped_constraint_(wrapped_constraint),
        basis_function_values_(basis_function_values) {}

  void DoEval(const Eigen::Ref<const Eigen::VectorXd>& x,
              Eigen::VectorXd* y) const override {
    AutoDiffVecXd y_t;
    Eval(initializeAutoDiff(x), &y_t);
    *y = autoDiffToValueMatrix(y_t);
  }

  void DoEval(const Eigen::Ref<const AutoDiffVecXd>& x,
              AutoDiffVecXd* y) const override {
    AutoDiffVecXd x_sum = basis_function_values_[0] *
                          x.segment(0, wrapped_constraint_->num_vars());
    const int num_terms = basis_function_values_.size();
    for (int i = 1; i < num_terms; ++i) {
      x_sum += basis_function_values_[i] *
               x.segment(i * wrapped_constraint_->num_vars(),
                         wrapped_constraint_->num_vars());
    }
    wrapped_constraint_->Eval(x_sum, y);
  }

  void DoEval(
      const Eigen::Ref<const drake::VectorX<drake::symbolic::Variable>>& x,
      drake::VectorX<drake::symbolic::Expression>* y) const override {
    throw std::runtime_error("PointConstraint on Expression not implemented");
  }

  int numOutputs() const { return wrapped_constraint_->num_outputs(); }

 private:
  std::shared_ptr<Constraint> wrapped_constraint_;
  std::vector<double> basis_function_values_;
};

}  // namespace

KinematicTrajectoryOptimization::KinematicTrajectoryOptimization(
    const math::BsplineCurve<double>& position_curve_seed)
    : num_positions_(position_curve_seed.rows()),
      placeholder_q_vars_(MakeNamedVariables("q", num_positions_)),
      placeholder_v_vars_(MakeNamedVariables("v", num_positions_)),
      placeholder_a_vars_(MakeNamedVariables("a", num_positions_)),
      placeholder_j_vars_(MakeNamedVariables("j", num_positions_)),
      placeholder_duration_var_(symbolic::Variable("duration")),
      position_curve_(position_curve_seed) {
  SetPositionCurve(position_curve_seed);
  SetSolverOption(SnoptSolver::id(), "Scale option", 1);
}

KinematicTrajectoryOptimization::KinematicTrajectoryOptimization(
    int num_positions, int num_control_points, int spline_order)
    : KinematicTrajectoryOptimization(math::BsplineCurve<double>(
          math::BsplineBasis(spline_order, num_control_points),
          std::vector<MatrixX<double>>(
              num_control_points, MatrixX<double>::Zero(num_positions, 1)))) {}

void KinematicTrajectoryOptimization::SetPositionCurve(
    const math::BsplineCurve<double>& position_curve_seed) {
  if (position_curve_seed.rows() != num_positions()) {
    throw std::logic_error("Invalid number of rows in position_curve_seed.");
  }
  if (position_curve_seed.cols() != 1) {
    throw std::logic_error("Invalid number of columns in position_curve_seed.");
  }
  position_curve_ = position_curve_seed;
  // Re-scale position_curve_ to be from 0 to 1.
  std::vector<double> normalized_knots;
  normalized_knots.reserve(position_curve_.knots().size());
  duration_ = position_curve_.knots().back();
  std::transform(position_curve_.knots().begin(), position_curve_.knots().end(),
                 std::back_inserter(normalized_knots),
                 [this](double knot) -> double { return knot / duration_; });
  position_curve_ = BsplineCurve<double>(
      BsplineBasis(position_curve_.order(), normalized_knots),
      position_curve_.control_points());
  SetupMathematicalProgram();
}

void KinematicTrajectoryOptimization::AddFixedPositionConstraint(
    const drake::VectorX<double>& desired_position, double plan_time) {
  DRAKE_ASSERT(desired_position.size() == num_positions());
  AddLinearConstraint(position() == desired_position, {{plan_time, plan_time}});
}

void KinematicTrajectoryOptimization::AddFixedVelocityConstraint(
    const drake::VectorX<double>& desired_velocity, double plan_time) {
  DRAKE_ASSERT(desired_velocity.size() == num_positions());
  AddLinearConstraint(velocity() == desired_velocity, {{plan_time, plan_time}});
}

void KinematicTrajectoryOptimization::AddFixedAccelerationConstraint(
    const drake::VectorX<double>& desired_acceleration, double plan_time) {
  DRAKE_ASSERT(desired_acceleration.size() == num_positions());
  AddLinearConstraint(acceleration() == desired_acceleration,
                      {{plan_time, plan_time}});
}

void KinematicTrajectoryOptimization::AddPositionBounds(
    const VectorX<double>& lower_bound, const VectorX<double>& upper_bound) {
  DRAKE_ASSERT(lower_bound.size() == num_positions());
  DRAKE_ASSERT(upper_bound.size() == num_positions());
  AddLinearConstraint(lower_bound <= position() && position() <= upper_bound);
}

void KinematicTrajectoryOptimization::AddVelocityBounds(
    const VectorX<double>& lower_bound, const VectorX<double>& upper_bound) {
  DRAKE_ASSERT(lower_bound.size() == num_positions());
  DRAKE_ASSERT(upper_bound.size() == num_positions());
  AddLinearConstraint(velocity() >= duration() * lower_bound &&
                      velocity() <= duration() * upper_bound);
}

void KinematicTrajectoryOptimization::AddDurationBounds(
    optional<double> lower_bound, optional<double> upper_bound) {
  if (lower_bound) {
    AddLinearConstraint(duration() >= *lower_bound, {{0., 0.}});
  }
  if (upper_bound) {
    AddLinearConstraint(duration() <= *upper_bound, {{0, 0}});
  }
}

void KinematicTrajectoryOptimization::AddDurationCost(double weight) {
  AddLinearCost(weight * duration());
}

void KinematicTrajectoryOptimization::AddVelocityCost(double weight) {
  for (int i = 0; i < num_positions(); ++i) {
    AddQuadraticCost(weight * velocity()(i) * velocity()(i));
  }
}

void KinematicTrajectoryOptimization::AddAccelerationCost(double weight) {
  for (int i = 0; i < num_positions(); ++i) {
    AddQuadraticCost(weight * acceleration()(i) * acceleration()(i));
  }
}

void KinematicTrajectoryOptimization::AddJerkCost(double weight) {
  for (int i = 0; i < num_positions(); ++i) {
    AddQuadraticCost(weight * jerk()(i) * jerk()(i));
  }
}

void KinematicTrajectoryOptimization::AddGenericPositionConstraint(
    const std::shared_ptr<Constraint>& constraint,
    const std::array<double, 2>& plan_interval,
    const std::shared_ptr<Constraint>& validation_constraint) {
  // Add the constraint to the vector in case we need to re-build later.
  if (plan_interval.back() - plan_interval.front() <
      PiecewiseTrajectory<double>::kEpsilonTime) {
    generic_position_constraints_.push_back(
        ConstraintWrapper({constraint,
                           plan_interval,
                           validation_constraint,
                           {plan_interval[0]}}));
  } else {
    const auto t = VectorX<double>::LinSpaced(
        initial_num_evaluation_points_, plan_interval[0], plan_interval[1]);
    std::set<double> evaluation_times;
    for (int i = 0; i < initial_num_evaluation_points_; ++i) {
      evaluation_times.insert(t(i));
    }
    generic_position_constraints_.push_back(ConstraintWrapper(
        {constraint, plan_interval, validation_constraint, evaluation_times}));
  }
  AddGenericPositionConstraintToProgram(generic_position_constraints_.back(),
                                        prog_.get_mutable());
}

void KinematicTrajectoryOptimization::AddLinearConstraint(
    const symbolic::Formula& f, const std::array<double, 2>& plan_interval) {
  formula_linear_constraints_.push_back(FormulaWrapper({f, plan_interval}));
  AddLinearConstraintToProgram(formula_linear_constraints_.back(),
                               prog_.get_mutable());
}

void KinematicTrajectoryOptimization::AddQuadraticCost(
    const symbolic::Expression& f, const std::array<double, 2>& plan_interval) {
  expression_quadratic_costs_.push_back(ExpressionWrapper({f, plan_interval}));
  AddQuadraticCostToProgram(expression_quadratic_costs_.back(),
                            prog_.get_mutable());
}

void KinematicTrajectoryOptimization::AddLinearCost(
    const symbolic::Expression& f, const std::array<double, 2>& plan_interval) {
  expression_linear_costs_.push_back(ExpressionWrapper({f, plan_interval}));
  AddLinearCostToProgram(expression_linear_costs_.back(), prog_.get_mutable());
}

std::vector<symbolic::Substitution>
KinematicTrajectoryOptimization::ConstructPlaceholderVariableSubstitution(
    const std::vector<MatrixXDecisionVariable>& control_points,
    const std::array<double, 2>& plan_interval) const {
  // Create symbolic curves
  math::BsplineCurve<symbolic::Expression> symbolic_q_curve{
      position_curve_.basis(), control_points};
  math::BsplineCurve<symbolic::Expression> symbolic_v_curve{
      symbolic_q_curve.Derivative()};
  math::BsplineCurve<symbolic::Expression> symbolic_a_curve{
      symbolic_v_curve.Derivative()};
  math::BsplineCurve<symbolic::Expression> symbolic_j_curve{
      symbolic_a_curve.Derivative()};

  std::vector<symbolic::Substitution> sub;
  if (plan_interval.back() - plan_interval.front() <
      trajectories::PiecewiseTrajectory<double>::kEpsilonTime) {
    sub.resize(1);
    sub[0].emplace(placeholder_duration_var_, duration_variable_(0));
    const VectorX<symbolic::Expression> symbolic_q_value =
        symbolic_q_curve.value(plan_interval.front());
    const VectorX<symbolic::Expression> symbolic_v_value =
        symbolic_v_curve.value(plan_interval.front());
    const VectorX<symbolic::Expression> symbolic_a_value =
        symbolic_a_curve.value(plan_interval.front());
    const VectorX<symbolic::Expression> symbolic_j_value =
        symbolic_j_curve.value(plan_interval.front());
    for (int i = 0; i < num_positions(); ++i) {
      sub[0].emplace(placeholder_q_vars_(i), symbolic_q_value(i));
      sub[0].emplace(placeholder_v_vars_(i), symbolic_v_value(i));
      sub[0].emplace(placeholder_a_vars_(i), symbolic_a_value(i));
      sub[0].emplace(placeholder_j_vars_(i), symbolic_j_value(i));
    }
  } else {
    const std::vector<int> active_control_point_indices{
        position_curve_.basis().ComputeActiveControlPointIndices(
            plan_interval)};
    sub.resize(active_control_point_indices.size());

    const int num_active_control_points = active_control_point_indices.size();
    for (int j = 0; j < num_active_control_points; ++j) {
      sub[j].emplace(placeholder_duration_var_, duration_variable_(0));
      for (int i = 0; i < num_positions(); ++i) {
        sub[j].emplace(
            placeholder_q_vars_(i),
            symbolic_q_curve.control_points()[active_control_point_indices[j]](
                i));
        if (0 < j) {
          sub[j].emplace(
              placeholder_v_vars_(i),
              symbolic_v_curve
                  .control_points()[active_control_point_indices[j - 1]](i)
                  .Expand());
        }
        if (1 < j) {
          sub[j].emplace(
              placeholder_a_vars_(i),
              symbolic_a_curve
                  .control_points()[active_control_point_indices[j - 2]](i)
                  .Expand());
        }
        if (2 < j) {
          sub[j].emplace(
              placeholder_j_vars_(i),
              symbolic_j_curve
                  .control_points()[active_control_point_indices[j - 3]](i)
                  .Expand());
        }
      }
    }
  }
  return sub;
}

std::vector<symbolic::Formula>
KinematicTrajectoryOptimization::SubstitutePlaceholderVariables(
    const symbolic::Formula& formula,
    const std::vector<MatrixXDecisionVariable>& control_points,
    const std::array<double, 2>& plan_interval) const {
  std::vector<symbolic::Formula> substitution_results;
  std::vector<symbolic::Substitution> substitutions =
      ConstructPlaceholderVariableSubstitution(control_points, plan_interval);
  substitution_results.reserve(substitutions.size());
  for (const auto& substitution : substitutions) {
    symbolic::Formula f{formula.Substitute(substitution)};
    // There are more substitutions for positions than velocities, etc. Check
    // that this formula doesn't contain any un-subsituted placeholers.
    if (!ContainsPlaceholders(f.GetFreeVariables())) {
      substitution_results.push_back(f);
    }
  }
  return substitution_results;
}

std::vector<symbolic::Expression>
KinematicTrajectoryOptimization::SubstitutePlaceholderVariables(
    const symbolic::Expression& expression,
    const std::vector<MatrixXDecisionVariable>& control_points,
    const std::array<double, 2>& plan_interval) const {
  std::vector<symbolic::Expression> substitution_results;
  std::vector<symbolic::Substitution> substitutions =
      ConstructPlaceholderVariableSubstitution(control_points, plan_interval);
  substitution_results.reserve(substitutions.size());
  for (const auto& substitution : substitutions) {
    symbolic::Expression e = expression.Substitute(substitution);
    // There are more substitutions for positions than velocities, etc. Check
    // that this expression doesn't contain any un-subsituted placeholers.
    if (!ContainsPlaceholders(e.GetVariables())) {
      substitution_results.push_back(e);
    }
  }
  return substitution_results;
}

bool KinematicTrajectoryOptimization::ContainsPlaceholders(
    const symbolic::Variables& vars) const {
  if (!symbolic::intersect(symbolic::Variables(placeholder_q_vars_), vars)
           .empty()) {
    return true;
  }
  if (!symbolic::intersect(symbolic::Variables(placeholder_v_vars_), vars)
           .empty()) {
    return true;
  }
  if (!symbolic::intersect(symbolic::Variables(placeholder_a_vars_), vars)
           .empty()) {
    return true;
  }
  if (!symbolic::intersect(symbolic::Variables(placeholder_j_vars_), vars)
           .empty()) {
    return true;
  }
  if (!symbolic::intersect(symbolic::Variables({placeholder_duration_var_}),
                           vars)
           .empty()) {
    return true;
  }
  return false;
}

void KinematicTrajectoryOptimization::AddLinearConstraintToProgram(
    const FormulaWrapper& constraint, MathematicalProgram* prog) const {
  DRAKE_ASSERT(prog);
  const std::vector<symbolic::Formula> per_control_point_formulae =
      SubstitutePlaceholderVariables(constraint.formula,
                                     control_point_variables_,
                                     constraint.plan_interval);
  for (const auto& f : per_control_point_formulae) {
    prog->AddLinearConstraint(f);
  }
}

void KinematicTrajectoryOptimization::AddQuadraticCostToProgram(
    const ExpressionWrapper& cost, MathematicalProgram* prog) const {
  DRAKE_ASSERT(prog);
  const std::vector<symbolic::Expression> per_control_point_expressions =
      SubstitutePlaceholderVariables(cost.expression, control_point_variables_,
                                     cost.plan_interval);
  for (const auto& expression : per_control_point_expressions) {
    prog->AddQuadraticCost(expression.Expand());
  }
}

void KinematicTrajectoryOptimization::AddLinearCostToProgram(
    const ExpressionWrapper& cost, MathematicalProgram* prog) const {
  DRAKE_ASSERT(prog);
  const std::vector<symbolic::Expression> per_control_point_expressions =
      SubstitutePlaceholderVariables(cost.expression, control_point_variables_,
                                     cost.plan_interval);
  for (const auto& expression : per_control_point_expressions) {
    prog->AddLinearCost(expression.Expand());
  }
}

void KinematicTrajectoryOptimization::AddGenericPositionConstraintToProgram(
    const ConstraintWrapper& constraint, MathematicalProgram* prog) const {
  DRAKE_ASSERT(prog);
  drake::log()->info("Adding generic position constraint at {} points.",
                     constraint.evaluation_times.size());
  for (const auto& time : constraint.evaluation_times) {
    AddPositionPointConstraintToProgram(constraint, time, prog);
  }
}

void KinematicTrajectoryOptimization::AddPositionPointConstraintToProgram(
    const ConstraintWrapper& constraint, double evaluation_time,
    MathematicalProgram* prog) const {
  std::vector<double> basis_function_values;
  basis_function_values.reserve(position_curve_.order());
  VectorXDecisionVariable var_vector(position_curve_.order() * num_positions());
  std::vector<int> active_control_point_indices =
      position_curve_.basis().ComputeActiveControlPointIndices(
          {{evaluation_time, evaluation_time}});
  for (int i = 0; i < position_curve_.order(); ++i) {
    const int control_point_index = active_control_point_indices[i];
    basis_function_values.push_back(
        position_curve_.basis().polynomials()[control_point_index].value(
            evaluation_time)(0));
    var_vector.segment(i * num_positions(), num_positions()) =
        control_point_variables_[control_point_index];
  }
  drake::log()->trace("Adding constraint at t = {}", evaluation_time);
  prog->AddConstraint(std::make_shared<PointConstraint>(constraint.constraint,
                                                        basis_function_values),
                      var_vector);
}

void KinematicTrajectoryOptimization::SetupMathematicalProgram() {
  auto new_prog = std::make_unique<MathematicalProgram>();
  if (prog_) {
    for (drake::solvers::SolverId id : {SnoptSolver::id()}) {
      for (const auto option : prog_->GetSolverOptionsDouble(id)) {
        new_prog->SetSolverOption(id, option.first, option.second);
      }
      for (const auto option : prog_->GetSolverOptionsInt(id)) {
        new_prog->SetSolverOption(id, option.first, option.second);
      }
      for (const auto option : prog_->GetSolverOptionsStr(id)) {
        new_prog->SetSolverOption(id, option.first, option.second);
      }
    }
  }
  prog_.swap(new_prog);
  duration_variable_ = prog_->NewContinuousVariables(1, "duration");
  control_point_variables_.clear();
  control_point_variables_.reserve(num_control_points());
  for (int i = 0; i < num_control_points(); ++i) {
    control_point_variables_.push_back(prog_->NewContinuousVariables(
        num_positions(), 1, "control_point_" + std::to_string(i)));
  }

  for (const auto& formula_constraint : formula_linear_constraints_) {
    AddLinearConstraintToProgram(formula_constraint, prog_.get_mutable());
  }

  for (const auto& expression_cost : expression_quadratic_costs_) {
    AddQuadraticCostToProgram(expression_cost, prog_.get_mutable());
  }

  for (const auto& expression_cost : expression_linear_costs_) {
    AddLinearCostToProgram(expression_cost, prog_.get_mutable());
  }

  for (const auto& generic_position_constraint :
       generic_position_constraints_) {
    AddGenericPositionConstraintToProgram(generic_position_constraint,
                                          prog_.get_mutable());
  }
}

SolutionResult KinematicTrajectoryOptimization::Solve(
    bool always_update_curve) {
  drake::log()->info("Num control points: {}", num_control_points());

  prog_->SetInitialGuess(duration_variable_, Vector1<double>(duration_));
  for (int i = 0; i < num_control_points(); ++i) {
    prog_->SetInitialGuess(control_point_variables_[i],
                           position_curve_.control_points()[i]);
  }

  SolutionResult result{SolutionResult::kUnknownError};
  if (formula_linear_constraints_.empty() &&
      expression_quadratic_costs_.empty() && expression_linear_costs_.empty() &&
      generic_position_constraints_.empty()) {
    result = SolutionResult::kSolutionFound;
  } else {
    result = prog_->Solve();
    drake::log()->info("Solver used: {}", prog_->GetSolverId().value().name());
  }

  if (always_update_curve || result == SolutionResult::kSolutionFound ||
      result == SolutionResult::kIterationLimit) {
    std::vector<MatrixX<double>> new_control_points;
    new_control_points.reserve(num_control_points());
    drake::log()->debug("Num control point variables: {}",
                        control_point_variables_.size());
    for (const auto& control_point_variable : control_point_variables_) {
      drake::log()->trace(
          "control point: {}",
          prog_->GetSolution(control_point_variable).transpose());
      new_control_points.push_back(prog_->GetSolution(control_point_variable));
    }
    position_curve_ =
        math::BsplineCurve<double>(position_curve_.basis(), new_control_points);
    duration_ = prog_->GetSolution(duration_variable_)(0);
  }

  return result;
}

optional<BsplineCurve<double>>
KinematicTrajectoryOptimization::ComputeFirstSolution(
    std::vector<KinematicTrajectoryOptimization>* programs,
    drake::optional<double> min_duration) {
  drake::log()->info("Calling program.Solve() ...");
  bool done{false};
  drake::solvers::SolutionResult result{
      drake::solvers::SolutionResult::kUnknownError};
  auto program = programs->begin();
  while (!done) {
    result = program->Solve(false);
    drake::log()->info("Solution result: {}", result);
    if (result == drake::solvers::SolutionResult::kSolutionFound) {
      done = !program->UpdateGenericConstraints();
    } else if (result != drake::solvers::SolutionResult::kIterationLimit) {
      done = !program->AddKnots();
    }
    if (!done && result != drake::solvers::SolutionResult::kSolutionFound &&
        result != drake::solvers::SolutionResult::kIterationLimit) {
      // Look for the next program in the vector that has no more control points
      // than the current program. If none exists, keep working on the current
      // program.
      auto next_program = std::next(program);
      while (next_program != program) {
        if (next_program == programs->end()) {
          next_program = programs->begin();
        }
        if (next_program->num_control_points() <=
            program->num_control_points()) {
          program = next_program;
          break;
        }
        ++next_program;
      }
    }
  }
  drake::log()->info("... Done.");

  // Has an optimized a trajectory.
  if (result == drake::solvers::SolutionResult::kSolutionFound) {
    BsplineCurve<double> position_curve{program->GetPositionCurve()};
    // Scale back to the original timing if it is specified, and the
    // optimized trajectory is shorter than it.
    const double opt_duration =
        position_curve.end_time() - position_curve.start_time();
    if (min_duration && (opt_duration < min_duration)) {
      return program->GetPositionCurve(*min_duration / opt_duration);
    }
    return position_curve;
  }
  return nullopt;
}

bool KinematicTrajectoryOptimization::UpdateGenericConstraints() {
  bool constraints_have_been_modified{false};
  for (auto& constraint : generic_position_constraints_) {
    drake::log()->debug(
        "Checking generic constraint over the interval [{}, {}]",
        constraint.plan_interval.front(), constraint.plan_interval.back());
    const VectorX<double> t{
        VectorX<double>::LinSpaced(num_validation_points_, 0., 1.)};
    int violation_start_index = -1;
    int violation_end_index = -1;
    for (int i = 0; i < num_validation_points_; ++i) {
      if (constraint.plan_interval.front() <= t(i) &&
          constraint.plan_interval.back() >= t(i)) {
        bool constraint_violated = false;
        if (constraint.validation_constraint) {
          constraint_violated =
              !constraint.validation_constraint->CheckSatisfied(
                  position_curve_.value(t(i)));
        } else {
          // TODO(avalenzu): Make this tolerance user-settable.
          constraint_violated = !constraint.constraint->CheckSatisfied(
              position_curve_.value(t(i)), 5e-3);
        }
        if (constraint_violated && i < num_validation_points_ - 1) {
          if (violation_start_index < 0) {
            violation_start_index = i;
          }
        } else if (violation_start_index > 0) {
          violation_end_index = i - 1;
          drake::log()->debug(
              "Generic constraint violated on the interval [{}, {}]",
              t(violation_start_index), t(violation_end_index));
          int violation_mid_index =
              violation_start_index +
              (violation_end_index - violation_start_index) / 2;
          bool new_time_inserted{false};
          const double& new_time{t(violation_mid_index)};
          std::tie(std::ignore, new_time_inserted) =
              constraint.evaluation_times.insert(new_time);
          if (new_time_inserted) {
            AddPositionPointConstraintToProgram(constraint, new_time,
                                                prog_.get_mutable());
            drake::log()->debug("Added point constraint at t = {}", new_time);
            constraints_have_been_modified = true;
          }
          violation_start_index = -1;
          violation_end_index = -1;
        }
      }
    }
  }
  return constraints_have_been_modified;
}

bool KinematicTrajectoryOptimization::AddKnots() {
  bool knots_have_been_added{false};
  const std::vector<double>& knots = position_curve_.knots();
  std::vector<double> additional_knots;
  const int num_knots = knots.size();
  for (int i = 0; i < num_knots - 1; ++i) {
    double new_knot = 0.5 * (knots[i] + knots[i + 1]);
    if (new_knot - knots[i] > min_knot_resolution()) {
      knots_have_been_added = true;
      additional_knots.push_back(new_knot);
    }
  }
  if (knots_have_been_added) {
    position_curve_.InsertKnot(additional_knots);
    SetupMathematicalProgram();
  }
  return knots_have_been_added;
}

math::BsplineCurve<double> KinematicTrajectoryOptimization::GetPositionCurve(
    double time_scaling) const {
  std::vector<double> scaled_knots;
  scaled_knots.reserve(position_curve_.knots().size());
  std::transform(position_curve_.knots().begin(), position_curve_.knots().end(),
                 std::back_inserter(scaled_knots),
                 [time_scaling, this](double knot) -> double {
                   return time_scaling * duration_ * knot;
                 });
  return math::BsplineCurve<double>(
      math::BsplineBasis(position_curve_.order(), scaled_knots),
      position_curve_.control_points());
}

PiecewisePolynomial<double>
KinematicTrajectoryOptimization::GetPositionTrajectory(
    double time_scaling) const {
  return *GetPositionCurve(time_scaling).piecewise_polynomial();
}

PiecewisePolynomial<double>
KinematicTrajectoryOptimization::GetPositionAndVelocityTrajectory(
    double time_scaling) const {
  PiecewisePolynomial<double> position_pp = GetPositionTrajectory(time_scaling);
  PiecewisePolynomial<double> velocity_pp = position_pp.derivative();
  std::vector<double> breaks = position_pp.get_segment_times();
  const int num_segments = position_pp.get_number_of_segments();
  const int num_positions = position_pp.rows();
  const int num_velocities = velocity_pp.rows();

  std::vector<PiecewisePolynomial<double>::PolynomialMatrix> polynomials;
  for (int i = 0; i < num_segments; ++i) {
    polynomials.emplace_back(num_positions + num_velocities, 1);
    polynomials.back() << position_pp.getPolynomialMatrix(i),
        velocity_pp.getPolynomialMatrix(i);
  }

  return PiecewisePolynomial<double>(polynomials, breaks);
}

}  // namespace planner
}  // namespace manipulation
}  // namespace drake
