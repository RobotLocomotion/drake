#include "drake/solvers/mathematical_program_result.h"

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

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
internal::SolverResult MathematicalProgramResult::ConvertToSolverResult()
    const {
  internal::SolverResult solver_result(solver_id_);
  if (x_val_.size() != 0) {
    solver_result.set_decision_variable_values(x_val_);
  }
  solver_result.set_optimal_cost(optimal_cost_);
  return solver_result;
}
#pragma GCC diagnostic pop

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
    const std::unordered_map<symbolic::Variable::Id, int>& variable_index,
    const Eigen::Ref<const Eigen::VectorXd>& variable_values) {
  DRAKE_DEMAND(variable_values.rows() ==
               static_cast<int>(variable_index.size()));
  auto it = variable_index.find(var.get_id());
  if (it == variable_index.end()) {
    std::stringstream oss;
    oss << "GetVariableValue: " << var
        << " is not captured by the decision_variable_index map, passed in "
           "set_decision_variable_index().";
    throw std::invalid_argument(oss.str());
  }
  return variable_values(it->second);
}

double MathematicalProgramResult::GetSolution(
    const symbolic::Variable& var) const {
  DRAKE_DEMAND(decision_variable_index_.has_value());
  return GetVariableValue(var, decision_variable_index_.value(), x_val_);
}

double MathematicalProgramResult::GetSuboptimalSolution(
    const symbolic::Variable& var, int solution_number) const {
  DRAKE_DEMAND(decision_variable_index_.has_value());
  return GetVariableValue(var, decision_variable_index_.value(),
                          suboptimal_x_val_[solution_number]);
}

void MathematicalProgramResult::AddSuboptimalSolution(
    double suboptimal_objective, const Eigen::VectorXd& suboptimal_x) {
  suboptimal_x_val_.push_back(suboptimal_x);
  suboptimal_objectives_.push_back(suboptimal_objective);
}
}  // namespace solvers
}  // namespace drake
