/* clang-format off to disable clang-format-includes */
#include "drake/solvers/ipopt_solver.h"
/* clang-format on */

#include <stdexcept>

namespace drake {
namespace solvers {

bool IpoptSolver::is_available() { return false; }

SolutionResult IpoptSolver::Solve(MathematicalProgram&) const {
  throw std::runtime_error(
      "The IPOPT bindings were not compiled.  You'll need to use a different "
      "solver.");
}

}  // namespace solvers
}  // namespace drake
