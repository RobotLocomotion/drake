#include "drake/solvers/solution_result.h"

namespace drake {
namespace solvers {
std::string to_string(SolutionResult solution_result) {
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
  // The following lines should not be reached, we add this line due to a defect
  // in the compiler.
  throw std::runtime_error("Should not reach here");
}

std::ostream& operator<<(std::ostream& os, SolutionResult solution_result) {
  os << to_string(solution_result);
  return os;
}
}  // namespace solvers
}  // namespace drake

