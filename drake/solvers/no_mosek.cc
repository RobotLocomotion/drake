
#include "drake/solvers/MosekSolver.h"

#include <stdexcept>

namespace drake {
namespace solvers {

bool MosekSolver::available() const {
  return false;
}

SolutionResult MosekSolver::Solve(OptimizationProblem &prog) const {
  throw std::runtime_error(
      "Mosek is not installed in your build. You'll need to use a different "
      "solver.");
}

}  // namespace drake
}  // namespace solvers
