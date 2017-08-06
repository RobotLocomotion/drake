/* clang-format off to disable clang-format-includes */
#include "drake/solvers/nlopt_solver.h"
/* clang-format on */

#include <stdexcept>

namespace drake {
namespace solvers {

bool NloptSolver::available() const {
  return false;
}

SolutionResult NloptSolver::Solve(MathematicalProgram&) const {
  throw std::runtime_error(
      "The Nlopt bindings were not compiled.  You'll need to use a different "
      "solver.");
}

}  // namespace solvers
}  // namespace drake
