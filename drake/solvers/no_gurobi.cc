#include "drake/solvers/gurobi_solver.h"

#include <stdexcept>

namespace drake {
namespace solvers {

bool GurobiSolver::available_impl() const { return false; }

GurobiSolverResult* GurobiSolver::Solve_impl(
    MathematicalProgram* const prog) const {
  throw std::runtime_error(
      "The Gurobi bindings were not compiled.  You'll need to use a different "
      "solver.");
}

}  // end namespace solvers
}  // end namespace drake
