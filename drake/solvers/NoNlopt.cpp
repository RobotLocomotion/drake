
#include <stdexcept>
#include "NloptSolver.h"

bool Drake::MathematicalProgramNloptSolver::available() const {
  return false;
}

bool Drake::MathematicalProgramNloptSolver::solve(
    OptimizationProblem &prog) const {
  throw std::runtime_error(
      "The Nlopt bindings were not compiled.  You'll need to use a different "
      "solver.");
}
