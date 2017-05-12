/* clang-format off */
#include "drake/solvers/mosek_solver.h"
/* clang-format on */

#include <stdexcept>

#include "drake/common/drake_assert.h"

namespace drake {
namespace solvers {

// Define no-op for license lock implementation.
class MosekLicenseLock::Impl {};
// Do not throw an error when a lock is constructed. Only do so when a user
// attempts to solve.
MosekLicenseLock::MosekLicenseLock() {}
MosekLicenseLock::~MosekLicenseLock() {}
MosekLicenseLock::Impl* MosekLicenseLock::impl() const {
  return nullptr;
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
