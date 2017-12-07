/* clang-format off to disable clang-format-includes */
#include "drake/solvers/mosek_solver.h"
/* clang-format on */

#include <stdexcept>

#include "drake/common/drake_assert.h"

using std::runtime_error;
using std::shared_ptr;

namespace drake {
namespace solvers {

shared_ptr<MosekSolver::License> MosekSolver::AcquireLicense() {
  return shared_ptr<MosekSolver::License>();
}

bool MosekSolver::available() const {
  return false;
}

SolutionResult MosekSolver::Solve(MathematicalProgram&) const {
  throw runtime_error(
      "Mosek is not installed in your build. You'll need to use a different "
      "solver.");
}

}  // namespace solvers
}  // namespace drake
