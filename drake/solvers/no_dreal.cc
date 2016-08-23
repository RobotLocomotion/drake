#include "drake/solvers/dreal_solver.h"

#include <stdexcept>

#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/optimization.h"

namespace drake {
namespace solvers {
bool DrealSolver::available() const { return false; }

SolutionResult DrealSolver::Solve(OptimizationProblem& prog) const {
  throw std::runtime_error(
      "The Dreal bindings were not compiled.  You'll need to use a different "
      "solver.");
}
}  // namespace drake
}  // namespace solvers
