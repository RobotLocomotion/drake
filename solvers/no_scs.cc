/* clang-format off to disable clang-format-includes */
#include "drake/solvers/scs_solver.h"
/* clang-format on */

#include <stdexcept>

namespace drake {
namespace solvers {

bool ScsSolver::is_available() {
  return false;
}

void ScsSolver::DoSolve2(const MathematicalProgram&, const Eigen::VectorXd&,
                         internal::SpecificOptions*,
                         MathematicalProgramResult*) const {
  throw std::runtime_error(
      "The SCS bindings were not compiled.  You'll need to use a different "
      "solver.");
}

}  // namespace solvers
}  // namespace drake
