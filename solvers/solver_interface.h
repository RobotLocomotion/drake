#pragma once

#include <optional>

#include <Eigen/Core>

#include "drake/common/drake_copyable.h"
#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/mathematical_program_result.h"
#include "drake/solvers/solution_result.h"
#include "drake/solvers/solver_id.h"
#include "drake/solvers/solver_options.h"

namespace drake {
namespace solvers {

/// Interface used by implementations of individual solvers.
class SolverInterface {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(SolverInterface)
  virtual ~SolverInterface();

  /// Returns true iff this solver was enabled at compile-time.
  virtual bool available() const = 0;

  /// Solves an optimization program with optional initial guess and solver
  /// options. Note that these initial guess and solver options are not written
  /// to @p prog.
  /// If the @p prog has set an option for a solver, and @p solver_options
  /// contains a different value for the same option on the same solver, then @p
  /// solver_options takes priority.
  virtual void Solve(const MathematicalProgram& prog,
                     const std::optional<Eigen::VectorXd>& initial_guess,
                     const std::optional<SolverOptions>& solver_options,
                     MathematicalProgramResult* result) const = 0;

  /// Returns the identifier of this solver.
  virtual SolverId solver_id() const = 0;

  /// Returns true if the program attributes are satisfied by the solver's
  /// capability.
  virtual bool AreProgramAttributesSatisfied(
      const MathematicalProgram& prog) const = 0;

 protected:
  SolverInterface();
};

}  // namespace solvers
}  // namespace drake
