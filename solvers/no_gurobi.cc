/* clang-format off to disable clang-format-includes */
#include "drake/solvers/gurobi_solver.h"
/* clang-format on */

#include <memory>
#include <stdexcept>

namespace drake {
namespace solvers {
std::shared_ptr<GurobiSolver::License> GurobiSolver::AcquireLicense() {
  return {};
}

bool GurobiSolver::is_available() {
  return false;
}

void GurobiSolver::DoSolve2(const MathematicalProgram&, const Eigen::VectorXd&,
                            internal::SpecificOptions*,
                            MathematicalProgramResult*) const {
  throw std::runtime_error(
      "The Gurobi bindings were not compiled.  You'll need to use a different "
      "solver.");
}

}  // namespace solvers
}  // namespace drake
