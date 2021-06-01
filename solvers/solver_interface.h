#pragma once

#include <optional>
#include <string>

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

  /// Returns true iff support for this solver has been compiled and linked
  /// into Drake. Certain solver implementations may have been excluded at
  /// compile-time due to licensing or linking restrictions.
  ///
  /// Contrast this with enabled(), which reflects whether a solver has been
  /// enabled or disabled at runtime (not compile-time).
  ///
  /// Typically only commercially-licensed solvers are ever not available() or
  /// not enabled().
  ///
  /// When this method returns false, the Solve method will throw.
  virtual bool available() const = 0;

  /// Returns true iff this solver is configured to be enabled at runtime. The
  /// mechanism to configure a particular solver implementation to be enabled
  /// or disabled at runtime (and which choice is the default when unspecified)
  /// is specific to the solver in question, but typically uses an environment
  /// variable.
  ///
  /// Contrast this with available(), which reflects whether a solver has been
  /// incorporated into Drake at compile-time (and has nothing to do with the
  /// runtime configuration). A solver that has been disabled at compile time
  /// (i.e., it is not available()) may still be configured to be available at
  /// runtime.
  ///
  /// Typically only commercially-licensed solvers are ever not available() or
  /// not enabled().
  ///
  /// When this method returns false, the Solve method will throw.
  virtual bool enabled() const = 0;

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

  /// Returns true iff the program's attributes are compatible with this
  /// solver's capabilities.
  virtual bool AreProgramAttributesSatisfied(
      const MathematicalProgram& prog) const = 0;

  /// Describes the reasons (if any) why the program is incompatible with this
  /// solver's capabilities. If AreProgramAttributesSatisfied would return true
  /// for the program, then this function returns the empty string.
  virtual std::string ExplainUnsatisfiedProgramAttributes(
      const MathematicalProgram& prog) const = 0;

 protected:
  SolverInterface();
};

}  // namespace solvers
}  // namespace drake
