/* clang-format off to disable clang-format-includes */
#include "drake/solvers/snopt_solver.h"
/* clang-format on */

#include <stdexcept>

namespace drake {
namespace solvers {

bool SnoptSolver::is_available() { return false; }

void SnoptSolver::DoSolve(
    const MathematicalProgram&,
    const Eigen::VectorXd&,
    const SolverOptions&,
    MathematicalProgramResult*) const {
  throw std::runtime_error(
      "The SNOPT bindings were not compiled.  You'll need to use a different "
      "solver.");
}

bool SnoptSolver::is_bounded_lp_broken() {
  throw std::runtime_error(
      "The SNOPT bindings were not compiled.  You'll need to use a different "
      "solver.");
}

}  // namespace solvers
}  // namespace drake
