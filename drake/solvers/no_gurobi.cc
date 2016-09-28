#include "gurobi_solver.h"

#include <stdexcept>

namespace drake {
namespace solvers {

bool GurobiSolver::available() const {
  return false;
}

SolutionResult GurobiSolver::Solve(MathematicalProgram& prog) const {
  throw std::runtime_error(
      "The Gurobi bindings were not compiled.  You'll need to use a different "
          "solver.");
}

}  // end namespace solvers
}  // end namespace drake
