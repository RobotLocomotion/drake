#include "drake/solvers/mathematical_program_result.h"

namespace drake {
namespace solvers {
MathematicalProgramResult::MathematicalProgramResult()
    : result_{SolutionResult::kUnknownError},
      x_val_{0},
      optimal_cost_{NAN},
      solver_id_{"unknown"},
      solver_details_{nullptr} {}

MathematicalProgramResult::MathematicalProgramResult(
    SolutionResult result, const Eigen::VectorXd& x_val, double optimal_cost,
    const SolverId& solver_id)
    : result_{result},
      x_val_{x_val},
      optimal_cost_{optimal_cost},
      solver_id_{solver_id},
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
}  // namespace solvers
}  // namespace drake
