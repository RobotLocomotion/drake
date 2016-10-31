#include "drake/solvers/dreal_solver.h"

#include <stdexcept>

#include "drake/solvers/mathematical_program.h"

namespace drake {
namespace solvers {
bool DrealSolver::available() const { return false; }

SolutionResult DrealSolver::Solve(MathematicalProgram& prog) const {
  throw std::runtime_error(
      "The Dreal bindings were not compiled.  You'll need to use a different "
      "solver.");
}
}  // namespace solvers
}  // namespace drake
