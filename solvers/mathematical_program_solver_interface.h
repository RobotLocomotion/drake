#pragma once

#include <ostream>
#include <string>

#include "drake/common/drake_copyable.h"
#include "drake/solvers/solver_id.h"

namespace drake {
namespace solvers {
class MathematicalProgram;

enum class SolutionResult {
  kSolutionFound,           ///< Found the optimal solution.
  kInvalidInput,           ///< Invalid input.
  kInfeasibleConstraints,  ///< The primal is infeasible.
  kUnbounded,              ///< The primal is unbounded.
  kUnknownError,           ///< Unknown error.
  kInfeasible_Or_Unbounded,  ///< The primal is either infeasible or unbounded.
  kIterationLimit,  ///< Reaches the iteration limits.
  kDualInfeasible,  ///< Dual problem is infeasible. In this case we cannot
                    /// infer the status of the primal problem.
};

std::string to_string(const SolutionResult& solution_result);
std::ostream& operator<<(std::ostream& os,
                         const SolutionResult& solution_result);

/// Interface used by implementations of individual solvers.
class MathematicalProgramSolverInterface {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MathematicalProgramSolverInterface)

  MathematicalProgramSolverInterface() = default;
  virtual ~MathematicalProgramSolverInterface() = default;

  /// Returns true iff this solver was enabled at compile-time.
  virtual bool available() const = 0;

  /// Sets values for the decision variables on the given MathematicalProgram
  /// @p prog, or:
  ///  * If no solver is available, throws std::runtime_error
  ///  * If the solver returns an error, returns a nonzero SolutionResult.
  // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
  virtual SolutionResult Solve(MathematicalProgram& prog) const = 0;

  /// Returns the identifier of this solver.
  virtual SolverId solver_id() const = 0;
};

}  // namespace solvers
}  // namespace drake
