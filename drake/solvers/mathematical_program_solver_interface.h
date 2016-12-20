#pragma once

#include <string>

namespace drake {
namespace solvers {
class MathematicalProgram;

enum SolutionResult {
  kSolutionFound = 0,
  kInvalidInput = -1,
  kInfeasibleConstraints = -2,
  kUnknownError = -3,
};

/// Interface used by implementations of individual solvers.
class MathematicalProgramSolverInterface {
 public:
  virtual ~MathematicalProgramSolverInterface() = default;

  /// Returns true iff this solver was enabled at compile-time.
  virtual bool available() const = 0;

  /// Returns the name of the solver.
  virtual std::string SolverName() const = 0;

  /// Sets values for the decision variables on the given MathematicalProgram
  /// @p prog, or:
  ///  * If no solver is available, throws std::runtime_error
  ///  * If the solver returns an error, returns a nonzero SolutionResult.
  // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
  virtual SolutionResult Solve(MathematicalProgram& prog) const = 0;
};
}  // namespace solvers
}  // namespace drake
