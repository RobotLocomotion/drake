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
    : solution_result_{SolutionResult::kUnknownError},
      x_val_{0},
      optimal_cost_{NAN},
      solver_id_{UnknownId()},
      solver_details_{
          systems::AbstractValue::Make<NoSolverDetails>(NoSolverDetails())} {}

const systems::AbstractValue& MathematicalProgramResult::get_solver_details()
    const {
  if (!solver_details_) {
    throw std::logic_error("The solver_details has not been set yet.");
  }
  return *solver_details_;
}

SolverResult MathematicalProgramResult::ConvertToSolverResult() const {
  SolverResult solver_result(solver_id_);
  if (x_val_.size() != 0) {
    solver_result.set_decision_variable_values(x_val_);
  }
  solver_result.set_optimal_cost(optimal_cost_);
  // This function doesn't set optimal_cost_lower_bound. If
  // SolverResult.optimal_cost_lower_bound needs to be set (like in
  // GurobiSolver), then the user will have to set it after calling
  // ConvertToSolverResult.
  return solver_result;
}
}  // namespace solvers
}  // namespace drake
