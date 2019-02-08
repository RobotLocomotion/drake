/* clang-format off to disable clang-format-includes */
#include "drake/solvers/mathematical_program.h"
/* clang-format on */

#include "drake/solvers/choose_best_solver.h"

// Note that the file mathematical_program_api.cc also contains some of the
// implementation of mathematical_program.h.

namespace drake {
namespace solvers {

SolutionResult MathematicalProgram::Solve() {
  const SolverId solver_id = ChooseBestSolver(*this);
  if (!solver_cache_for_deprecated_solve_method_ ||
      (solver_cache_for_deprecated_solve_method_->solver_id() != solver_id)) {
    solver_cache_for_deprecated_solve_method_ = MakeSolver(solver_id);
  }
  return solver_cache_for_deprecated_solve_method_->Solve(*this);
}

}  // namespace solvers
}  // namespace drake
