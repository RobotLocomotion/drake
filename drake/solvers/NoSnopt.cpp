
#include <stdexcept>
#include "SnoptSolver.h"

bool Drake::SnoptSolver::available() const {
  return false;
}

bool Drake::SnoptSolver::solve(
    OptimizationProblem &prog) const {
  throw std::runtime_error(
      "The SNOPT bindings were not compiled.  You'll need to use a different "
      "solver.");
}
