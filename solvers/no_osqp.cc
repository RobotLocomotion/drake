/* clang-format off to disable clang-format-includes */
#include "drake/solvers/osqp_solver.h"
/* clang-format on */

#include <stdexcept>

#include "drake/solvers/mathematical_program.h"

namespace drake {
namespace solvers {

bool OsqpSolver::is_available() { return false; }

SolutionResult OsqpSolver::Solve(MathematicalProgram&) const {
  throw std::runtime_error(
      "The OSQP bindings were not compiled.  You'll need to use a different "
      "solver.");
}

void OsqpSolver::Solve(const MathematicalProgram&,
                       const optional<Eigen::VectorXd>&,
                       const optional<SolverOptions>&,
                       MathematicalProgramResult*) const {
  throw std::runtime_error(
      "The OSQP bindings were not compiled.  You'll need to use a different "
      "solver.");
}

}  // namespace solvers
}  // namespace drake
