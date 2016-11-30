
#include "drake/solvers/ipopt_solver.h"

#include <stdexcept>

namespace drake {
namespace solvers {

bool IpoptSolver::available() const {
  return false;
}

SolutionResult IpoptSolver::Solve(MathematicalProgram &prog) const {
  throw std::runtime_error(
      "The IPOPT bindings were not compiled.  You'll need to use a different "
      "solver.");
}

}  // namespace solvers
}  // namespace drake
