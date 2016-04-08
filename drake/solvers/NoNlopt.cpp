
#include "drake/solvers/NloptSolver.h"

#include <stdexcept>

bool Drake::NloptSolver::available() const {
  return false;
}

bool Drake::NloptSolver::Solve(
    OptimizationProblem &prog) const {
  throw std::runtime_error(
      "The Nlopt bindings were not compiled.  You'll need to use a different "
      "solver.");
}
