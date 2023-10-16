#pragma once

#include <ostream>
#include <string>

#include "drake/common/fmt.h"

namespace drake {
namespace solvers {
enum SolutionResult {
  /** Found the optimal solution. */
  kSolutionFound = 0,
  /** Invalid input. */
  kInvalidInput = -1,
  /** The primal is infeasible. */
  kInfeasibleConstraints = -2,
  /** The primal is unbounded. */
  kUnbounded = -3,
  /** Solver-specific error. (Try
  MathematicalProgramResult::get_solver_details() or enabling verbose solver
  output.) */
  kSolverSpecificError = -4,
  /** The primal is either infeasible or unbounded. */
  kInfeasibleOrUnbounded = -5,
  /** Reaches the iteration limits. */
  kIterationLimit = -6,
  /** Dual problem is infeasible. In this case we cannot infer the status of the
  primal problem. */
  kDualInfeasible = -7,
  /** The initial (invalid) solution result. This value should be overwritten by
  the solver during Solve(). */
  kSolutionResultNotSet = -8
};

std::string to_string(SolutionResult solution_result);
std::ostream& operator<<(std::ostream& os, SolutionResult solution_result);
}  // namespace solvers
}  // namespace drake

DRAKE_FORMATTER_AS(, drake::solvers, SolutionResult, x,
                   drake::solvers::to_string(x))
