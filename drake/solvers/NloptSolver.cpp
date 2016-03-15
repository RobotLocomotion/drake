
#include <stdexcept>
#include "NloptSolver.h"

bool Drake::MathematicalProgramNloptSolver::available() const {
  // TODO make this "true" once the interface exists.
  return false;
}

bool Drake::MathematicalProgramNloptSolver::solve(
    OptimizationProblem &prog) const {

  // TODO implement this.
  throw std::runtime_error("Nlopt bindings not yet implemented.");
}
