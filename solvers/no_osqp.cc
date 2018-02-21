/* clang-format off to disable clang-format-includes */
#include "drake/solvers/osqp_solver.h"
/* clang-format on */

#include <stdexcept>

#include "drake/solvers/mathematical_program.h"

namespace drake {
namespace solvers {

bool OsqpSolver::available() const { return false; }

SolutionResult OsqpSolver::Solve(MathematicalProgram&) const {
  throw std::runtime_error(
      "The OSQP bindings were not compiled.  You'll need to use a different "
      "solver.");
}

}  // namespace solvers
}  // namespace drake
