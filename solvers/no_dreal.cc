/* clang-format off to disable clang-format-includes */
#include "drake/solvers/dreal_solver.h"
/* clang-format on */

#include <stdexcept>

namespace drake {
namespace solvers {

bool DrealSolver::is_available() { return false; }

void DrealSolver::DoSolve(
    const MathematicalProgram&,
    const Eigen::VectorXd&,
    const SolverOptions&,
    MathematicalProgramResult*) const {
  throw std::runtime_error(
      "The dReal bindings were not compiled.  You'll need to use a different "
      "solver.");
}

}  // namespace solvers
}  // namespace drake
