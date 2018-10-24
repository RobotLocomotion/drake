#include "drake/solvers/mathematical_program_result.h"

#include <utility>

namespace drake {
namespace solvers {
MathematicalProgramResult::MathematicalProgramResult()
    : result_{SolutionResult::kUnknownError},
      x_val_{0},
      optimal_cost_{NAN},
      solver_id_{"unknown"},
      solver_details_{
          systems::AbstractValue::Make<NoSolverDetails>(NoSolverDetails())} {}

void MathematicalProgramResult::SetSolverDetails(
    std::unique_ptr<systems::AbstractValue> solver_details) {
  solver_details_ = std::move(solver_details);
}

const systems::AbstractValue& MathematicalProgramResult::solver_details()
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
  return solver_result;
}
}  // namespace solvers
}  // namespace drake
