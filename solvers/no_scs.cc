/* clang-format off to disable clang-format-includes */
#include "drake/solvers/scs_solver.h"
/* clang-format on */

#include <stdexcept>

#include "drake/solvers/mathematical_program.h"

namespace drake {
namespace solvers {

bool ScsSolver::is_available() { return false; }

void ScsSolver::Solve(const MathematicalProgram&,
                      const optional<Eigen::VectorXd>&,
                      const optional<SolverOptions>&,
                      MathematicalProgramResult*) const {
  throw std::runtime_error(
      "The SCS bindings were not compiled.  You'll need to use a different "
      "solver.");
}

SolutionResult ScsSolver::Solve(MathematicalProgram&) const {
  throw std::runtime_error(
      "The SCS bindings were not compiled.  You'll need to use a different "
      "solver.");
}

}  // namespace solvers
}  // namespace drake
