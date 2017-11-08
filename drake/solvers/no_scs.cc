/* clang-format off to disable clang-format-includes */
#include "drake/solvers/scs_solver.h"
/* clang-format on */

#include <stdexcept>

#include "drake/solvers/mathematical_program.h"

namespace drake {
namespace solvers {

bool ScsSolver::available() const { return false; }

SolutionResult ScsSolver::Solve(MathematicalProgram&) const {
  throw std::runtime_error(
      "The SCS bindings were not compiled.  You'll need to use a different "
      "solver.");
}

}  // namespace solvers
}  // namespace drake
