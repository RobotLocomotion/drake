
#include "drake/solvers/snopt_solver.h"

#include <stdexcept>

namespace drake {
namespace solvers {

bool SnoptSolver::available() const {
  return false;
}

SolutionResult SnoptSolver::Solve(MathematicalProgram &prog) const {
  throw std::runtime_error(
      "The SNOPT bindings were not compiled.  You'll need to use a different "
      "solver.");
}

}  // namespace drake
}  // namespace solvers
