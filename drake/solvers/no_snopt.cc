
#include "drake/solvers/snopt_solver.h"

#include <stdexcept>

namespace drake {
namespace solvers {

bool SnoptSolver::available_impl() const {
  return false;
}

SnoptSolverResult* SnoptSolver::Solve_impl(MathematicalProgram &prog) const {
  throw std::runtime_error(
      "The SNOPT bindings were not compiled.  You'll need to use a different "
      "solver.");
}

}  // namespace solvers
}  // namespace drake
