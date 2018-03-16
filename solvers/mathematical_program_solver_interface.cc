#include "drake/solvers/mathematical_program_solver_interface.h"

namespace drake {
namespace solvers {
std::string to_string(const SolutionResult& solution_result) {
  switch (solution_result) {
    case SolutionResult::kSolutionFound:
      return "SolutionFound";
    case SolutionResult::kInvalidInput:
      return "InvalidInput";
    case SolutionResult::kInfeasibleConstraints:
      return "InfeasibleConstraints";
    case SolutionResult::kUnbounded:
      return "Unbounded";
    case SolutionResult::kInfeasible_Or_Unbounded:
      return "Infeasible_Or_Unbounded";
    case SolutionResult::kIterationLimit:
      return "IterationLimit";
    case SolutionResult::kUnknownError:
      return "UnknownError";
    case SolutionResult::kDualInfeasible:
      return "DualInfeasible";
  }
}

std::ostream& operator<<(std::ostream& os,
                         const SolutionResult& solution_result) {
  os << to_string(solution_result);
  return os;
}
}  // namespace solvers
}  // namespace drake
