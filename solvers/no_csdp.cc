/* clang-format off to disable clang-format-includes */
#include "drake/solvers/csdp_solver.h"
/* clang-format on */

#include <stdexcept>

namespace drake {
namespace solvers {

bool CsdpSolver::is_available() { return false; }

void CsdpSolver::DoSolve(const MathematicalProgram&, const Eigen::VectorXd&,
                         const SolverOptions&,
                         MathematicalProgramResult*) const {
  throw std::runtime_error(
      "The CSDP bindings were not compiled.  You'll need to use a different "
      "solver.");
}

}  // namespace solvers
}  // namespace drake
