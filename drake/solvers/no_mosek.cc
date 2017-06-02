/* clang-format off */
#include "drake/solvers/mosek_solver.h"
/* clang-format on */

#include <stdexcept>

#include "drake/common/drake_assert.h"

namespace drake {
namespace solvers {

std::shared_ptr<MosekSolver::License> MosekSolver::AcquireLicense() {
  throw std::runtime_error(
      "Mosek is not installed in your build. You'll need to use a different "
      "solver.");
}

bool MosekSolver::available() const {
  return false;
}

SolutionResult MosekSolver::Solve(MathematicalProgram&) const {
  throw std::runtime_error(
      "Mosek is not installed in your build. You'll need to use a different "
      "solver.");
}

}  // namespace solvers
}  // namespace drake
