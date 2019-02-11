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
      solver_details_type_{nullptr},
      solver_details_{nullptr} {}

const AbstractValue& MathematicalProgramResult::get_solver_details() const {
  if (!solver_details_) {
    throw std::logic_error("The solver_details has not been set yet.");
  }
  return *solver_details_;
}

bool MathematicalProgramResult::is_success() const {
  return solution_result_ == SolutionResult::kSolutionFound;
}

SolverResult MathematicalProgramResult::ConvertToSolverResult() const {
  SolverResult solver_result(solver_id_);
  if (x_val_.size() != 0) {
    solver_result.set_decision_variable_values(x_val_);
  }
  solver_result.set_optimal_cost(optimal_cost_);
  return solver_result;
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

double MathematicalProgramResult::GetSolution(
    const symbolic::Variable& var) const {
  DRAKE_DEMAND(decision_variable_index_.has_value());
  auto it = decision_variable_index_->find(var.get_id());
  if (it == decision_variable_index_->end()) {
    std::stringstream oss;
    oss << "MathematicalProgramResult::GetSolution, " << var
        << " is not captured by the decision_variable_index map, passed in "
           "set_decision_variable_index().";
    throw std::invalid_argument(oss.str());
  }
  DRAKE_DEMAND(x_val_.size() ==
               static_cast<int>(decision_variable_index_->size()));
  return x_val_[it->second];
}
}  // namespace solvers
}  // namespace drake
