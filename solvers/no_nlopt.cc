/* clang-format off to disable clang-format-includes */
#include "drake/solvers/nlopt_solver.h"
/* clang-format on */

#include <stdexcept>

namespace drake {
namespace solvers {

bool NloptSolver::is_available() {
  return false;
}

void NloptSolver::DoSolve2(const MathematicalProgram&, const Eigen::VectorXd&,
                           internal::SpecificOptions*,
                           MathematicalProgramResult*) const {
  throw std::runtime_error(
      "The Nlopt bindings were not compiled.  You'll need to use a different "
      "solver.");
}
}  // namespace solvers
}  // namespace drake
