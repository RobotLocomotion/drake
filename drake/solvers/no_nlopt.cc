
#include "drake/solvers/nlopt_solver.h"

#include <stdexcept>

namespace drake {
namespace solvers {

bool NloptSolver::available() const {
  return false;
}

SolutionResult NloptSolver::Solve(MathematicalProgram &prog) const {
  throw std::runtime_error(
      "The Nlopt bindings were not compiled.  You'll need to use a different "
      "solver.");
}

}  // namespace solvers
}  // namespace drake
