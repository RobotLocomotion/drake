/* clang-format off to disable clang-format-includes */
#include "drake/solvers/daqp_solver.h"
/* clang-format on */

#include <stdexcept>

namespace drake {
namespace solvers {

bool DaqpSolver::is_available() {
  return false;
}

void DaqpSolver::DoSolve2(const MathematicalProgram&, const Eigen::VectorXd&,
                          internal::SpecificOptions*,
                          MathematicalProgramResult*) const {
  throw std::runtime_error(
      "The DAQP bindings were not compiled.  You'll need to use a different "
      "solver.");
}

}  // namespace solvers
}  // namespace drake
