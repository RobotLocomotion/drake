
#include "drake/solvers/IpoptSolver.h"

#include <stdexcept>

namespace drake {
namespace solvers {

bool IpoptSolver::available() const {
  return false;
}

SolutionResult IpoptSolver::Solve(OptimizationProblem &prog) const {
  throw std::runtime_error(
      "The IPOPT bindings were not compiled.  You'll need to use a different "
      "solver.");
}

}  // namespace drake
}  // namespace solvers
