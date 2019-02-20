/* clang-format off to disable clang-format-includes */
#include "drake/solvers/mathematical_program.h"
/* clang-format on */

#include "drake/solvers/choose_best_solver.h"

namespace drake {
namespace solvers {

// This method is placed in this file to break a dependency cycle.  In the
// MathematicalProgram, ideally we should only deal with problem formulation,
// and then in a SolverInterface implementation we use the MathematicalProgram
// as input to search for a solution.  However, the contract of this method *on
// MathematicalProgram* is to solve the program, which means it needs to depend
// on specific SolverInterface implementations, but those implementations need
// to depend on this program, so we have a cycle.  To break the cycle, we can
// compile this method in a separate file so that this method can depend on
// both MathematicalProgram and all of the SolverInterface implementations.
//
// Since this method is slated to be deprecated (#9633), this code-organization
// oddity won't have to last very long.
SolutionResult MathematicalProgram::Solve() {
  const SolverId solver_id = ChooseBestSolver(*this);
  // Note that this implementation isn't threadsafe (we might race on the
  // solver_cache_... value), but that's OK because the contract of Solve() is
  // already known to be racy and single-threaded only (it writes back the
  // solution results using setters on this MathematicalProgram instance).
  if (!solver_cache_for_deprecated_solve_method_ ||
      (solver_cache_for_deprecated_solve_method_->solver_id() != solver_id)) {
    solver_cache_for_deprecated_solve_method_ = MakeSolver(solver_id);
  }
  return solver_cache_for_deprecated_solve_method_->Solve(*this);
}

}  // namespace solvers
}  // namespace drake
