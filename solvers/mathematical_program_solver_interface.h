#pragma once

#include <Eigen/Core>

#include "drake/common/drake_copyable.h"
#include "drake/solvers/mathematical_program_result.h"
#include "drake/solvers/solution_result.h"
#include "drake/solvers/solver_id.h"
#include "drake/solvers/solver_options.h"
#include "drake/solvers/solver_result.h"

namespace drake {
namespace solvers {
class MathematicalProgram;

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

  /// Solves an optimization program with optional initial guess and solver
  /// options. Note that these initial guess and solver options are not written
  /// to @p prog.
  virtual void Solve(const MathematicalProgram& prog,
                     const optional<Eigen::VectorXd>& initial_guess,
                     const optional<SolverOptions>& solver_options,
                     MathematicalProgramResult* result) const = 0;

  /// Returns the identifier of this solver.
  virtual SolverId solver_id() const = 0;

  /// Returns true if the program attributes are satisfied by the solver's
  /// capability.
  virtual bool AreProgramAttributesSatisfied(
      const MathematicalProgram& prog) const = 0;
};

}  // namespace solvers
}  // namespace drake
