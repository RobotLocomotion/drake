
#include "drake/solvers/SnoptSolver.h"

#include <stdexcept>

bool Drake::SnoptSolver::available() const {
  return false;
}

bool Drake::SnoptSolver::Solve(
    OptimizationProblem &prog) const {
  throw std::runtime_error(
      "The SNOPT bindings were not compiled.  You'll need to use a different "
      "solver.");
}
