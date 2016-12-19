
#include "drake/solvers/nlopt_solver.h"

#include <stdexcept>

namespace drake {
namespace solvers {

bool NloptSolver::available_impl() const {
  return false;
}

NloptSolverResult* NloptSolver::Solve_impl(MathematicalProgram &prog) const {
  throw std::runtime_error(
      "The Nlopt bindings were not compiled.  You'll need to use a different "
      "solver.");
}

}  // namespace solvers
}  // namespace drake
