
#include "drake/solvers/IpoptSolver.h"

#include <stdexcept>

bool drake::solvers::IpoptSolver::available() const {
  return false;
}

drake::solvers::SolutionResult drake::solvers::IpoptSolver::Solve(
    Drake::OptimizationProblem &prog) const {
  throw std::runtime_error(
      "The IPOPT bindings were not compiled.  You'll need to use a different "
      "solver.");
}
