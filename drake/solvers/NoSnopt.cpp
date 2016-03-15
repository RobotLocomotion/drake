
#include <stdexcept>
#include "SnoptSolver.h"

bool Drake::SNOPTSolver::available() const {
  return false;
}

bool Drake::SNOPTSolver::solve(
    OptimizationProblem &prog) const {
  throw std::runtime_error(
      "The SNOPT bindings were not compiled.  You'll need to use a different "
      "solver.");
}
