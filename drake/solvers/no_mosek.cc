
#include "drake/solvers/mosek_solver.h"

#include <stdexcept>

namespace drake {
namespace solvers {

bool MosekSolver::available_impl() const {
  return false;
}

MosekSolverResult* MosekSolver::Solve_impl(MathematicalProgram &prog) const {
  throw std::runtime_error(
      "Mosek is not installed in your build. You'll need to use a different "
      "solver.");
}

}  // namespace solvers
}  // namespace drake
