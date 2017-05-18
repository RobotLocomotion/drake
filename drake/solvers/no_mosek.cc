/* clang-format off */
#include "drake/solvers/mosek_solver.h"
/* clang-format on */

#include <stdexcept>

#include "drake/common/drake_assert.h"

namespace drake {
namespace solvers {

MosekSolver::~MosekSolver() {
  DRAKE_ASSERT(mosek_env_ == nullptr);
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
