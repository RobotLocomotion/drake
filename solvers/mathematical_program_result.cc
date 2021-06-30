#include "drake/solvers/mathematical_program_result.h"

#include <fmt/format.h>

#include "drake/common/never_destroyed.h"

namespace drake {
namespace solvers {
namespace {
SolverId UnknownId() {
  static const never_destroyed<SolverId> result(SolverId({}));
  return result.access();
}
}  // namespace

MathematicalProgramResult::MathematicalProgramResult()
    : decision_variable_index_{},
      solution_result_{SolutionResult::kUnknownError},
      x_val_{0},
      optimal_cost_{NAN},
      solver_id_{UnknownId()},
      solver_details_{nullptr} {}

const AbstractValue& MathematicalProgramResult::get_abstract_solver_details()
    const {
  if (!solver_details_) {
    throw std::logic_error("The solver_details has not been set yet.");
  }
  return *solver_details_;
}

bool MathematicalProgramResult::is_success() const {
  return solution_result_ == SolutionResult::kSolutionFound;
}

void MathematicalProgramResult::set_x_val(const Eigen::VectorXd& x_val) {
  DRAKE_DEMAND(decision_variable_index_.has_value());
  if (x_val.size() != static_cast<int>(decision_variable_index_->size())) {
    std::stringstream oss;
    oss << "MathematicalProgramResult::set_x_val, the dimension of x_val is "
        << x_val.size() << ", expected " << decision_variable_index_->size();
    throw std::invalid_argument(oss.str());
  }
  x_val_ = x_val;
}

double GetVariableValue(
    const symbolic::Variable& var,
    const std::optional<std::unordered_map<symbolic::Variable::Id, int>>&
        variable_index,
    const Eigen::Ref<const Eigen::VectorXd>& variable_values) {
  DRAKE_ASSERT(variable_index.has_value());
  DRAKE_ASSERT(variable_values.rows() ==
               static_cast<int>(variable_index->size()));
  auto it = variable_index->find(var.get_id());
  if (it == variable_index->end()) {
    throw std::invalid_argument(fmt::format(
        "GetVariableValue: {} is not captured by the variable_index map.",
        var.get_name()));
  }
  return variable_values(it->second);
}

double MathematicalProgramResult::GetSolution(
    const symbolic::Variable& var) const {
  return GetVariableValue(var, decision_variable_index_, x_val_);
}

symbolic::Expression MathematicalProgramResult::GetSolution(
    const symbolic::Expression& e) const {
  DRAKE_ASSERT(decision_variable_index_.has_value());
  symbolic::Environment env;
  for (const auto& var : e.GetVariables()) {
    const auto it = decision_variable_index_->find(var.get_id());
    // We do not expect every variable to be in GetSolution (e.g. not the
    // indeterminates).
    if (it != decision_variable_index_->end()) {
      env.insert(var, x_val_(it->second));
    }
  }
  return e.EvaluatePartial(env);
}

symbolic::Polynomial MathematicalProgramResult::GetSolution(
    const symbolic::Polynomial& p) const {
  DRAKE_ASSERT(decision_variable_index_.has_value());
  for (const auto& indeterminate : p.indeterminates()) {
    if (decision_variable_index_->count(indeterminate.get_id()) > 0) {
      throw std::invalid_argument(
          fmt::format("GetSolution: {} is an indeterminate in the polynomial, "
                      "but result stores its value as a decision variable.",
                      indeterminate.get_name()));
    }
  }

  symbolic::Environment env;
  symbolic::Polynomial::MapType monomial_to_coefficient_result_map;
  for (const auto& [monomial, coefficient] : p.monomial_to_coefficient_map()) {
    for (const auto& var : coefficient.GetVariables()) {
      const auto it = decision_variable_index_->find(var.get_id());
      if (it != decision_variable_index_->end()) {
        env.insert(var, x_val_(it->second));
      }
    }
    // Evaluate the coefficient using env, and then add the pair (monomial,
    // coefficient_evaluate_result) to the new map.
    monomial_to_coefficient_result_map.emplace_hint(
        monomial_to_coefficient_result_map.end(), monomial,
        coefficient.EvaluatePartial(env));
  }
  return symbolic::Polynomial(monomial_to_coefficient_result_map);
}

double MathematicalProgramResult::GetSuboptimalSolution(
    const symbolic::Variable& var, int solution_number) const {
  return GetVariableValue(var, decision_variable_index_,
                          suboptimal_x_val_[solution_number]);
}

void MathematicalProgramResult::AddSuboptimalSolution(
    double suboptimal_objective, const Eigen::VectorXd& suboptimal_x) {
  suboptimal_x_val_.push_back(suboptimal_x);
  suboptimal_objectives_.push_back(suboptimal_objective);
}

std::vector<std::string>
MathematicalProgramResult::GetInfeasibleConstraintNames(
    const MathematicalProgram& prog, std::optional<double> tolerance) const {
  std::vector<std::string> descriptions;

  if (!tolerance) {
    // TODO(russt): Extract the constraint tolerance from the solver.  This
    // value was used successfully for some time in MATLAB Drake, so I've
    // ported it as the default here.
    tolerance = 1e-4;
  }

  for (const auto& binding : prog.GetAllConstraints()) {
    const Eigen::VectorXd val = this->EvalBinding(binding);
    const std::shared_ptr<Constraint>& constraint = binding.evaluator();
    std::string d = constraint->get_description();
    if (d.empty()) {
      d = NiceTypeName::Get(*constraint);
    }
    for (int i = 0; i < val.rows(); i++) {
      if (std::isnan(val(i)) ||
          val[i] < constraint->lower_bound()[i] - *tolerance ||
          val[i] > constraint->upper_bound()[i] + *tolerance) {
        descriptions.push_back(d + "[" + std::to_string(i) + "]: " +
                               std::to_string(constraint->lower_bound()[i]) +
                               " <= " + std::to_string(val[i]) + " <= " +
                               std::to_string(constraint->upper_bound()[i]));
      }
    }
  }
  return descriptions;
}

std::vector<Binding<Constraint>>
MathematicalProgramResult::GetInfeasibleConstraints(
    const MathematicalProgram& prog, std::optional<double> tolerance) const {
  std::vector<Binding<Constraint>> infeasible_bindings;

  if (!tolerance) {
    // TODO(russt): Extract the constraint tolerance from the solver.  This
    // value was used successfully for some time in MATLAB Drake, so I've
    // ported it as the default here.
    tolerance = 1e-4;
  }

  for (const auto& binding : prog.GetAllConstraints()) {
    const Eigen::VectorXd val = this->EvalBinding(binding);
    const std::shared_ptr<Constraint>& constraint = binding.evaluator();
    for (int i = 0; i < constraint->num_constraints(); ++i) {
      if (std::isnan(val(i)) ||
          val(i) > constraint->upper_bound()(i) + *tolerance ||
          val(i) < constraint->lower_bound()(i) - *tolerance) {
        infeasible_bindings.push_back(binding);
        continue;
      }
    }
  }
  return infeasible_bindings;
}
}  // namespace solvers
}  // namespace drake
