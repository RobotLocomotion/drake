/* clang-format off to disable clang-format-includes */
#include "drake/solvers/clarabel_solver.h"
/* clang-format on */

#include <stdexcept>

namespace drake {
namespace solvers {
bool ClarabelSolver::is_available() {
  return false;
}

void ClarabelSolver::DoSolve2(const MathematicalProgram&,
                              const Eigen::VectorXd&,
                              internal::SpecificOptions*,
                              MathematicalProgramResult*) const {
  throw std::runtime_error(
      "The Clarabel bindings were not compiled. You'll need to use a "
      "different solver.");
}
}  // namespace solvers
}  // namespace drake
