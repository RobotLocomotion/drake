
#include "drake/solvers/ipopt_solver.h"

#include <stdexcept>

namespace drake {
namespace solvers {

bool IpoptSolver::available_impl() const { return false; }

IpoptSolverResult* IpoptSolver::Solve_impl(
    MathematicalProgram* const prog) const {
  throw std::runtime_error(
      "The IPOPT bindings were not compiled.  You'll need to use a different "
      "solver.");
}

}  // namespace solvers
}  // namespace drake
