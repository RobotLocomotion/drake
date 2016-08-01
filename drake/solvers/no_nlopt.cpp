
#include "drake/solvers/NloptSolver.h"

#include <stdexcept>

namespace drake {
namespace solvers {

bool NloptSolver::available() const {
  return false;
}

SolutionResult NloptSolver::Solve(OptimizationProblem &prog) const {
  throw std::runtime_error(
      "The Nlopt bindings were not compiled.  You'll need to use a different "
      "solver.");
}

}  // namespace drake
}  // namespace solvers
