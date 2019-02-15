/* clang-format off to disable clang-format-includes */
#include "drake/solvers/mosek_solver.h"
/* clang-format on */

#include <stdexcept>

using std::runtime_error;
using std::shared_ptr;

namespace drake {
namespace solvers {

shared_ptr<MosekSolver::License> MosekSolver::AcquireLicense() {
  return shared_ptr<MosekSolver::License>();
}

bool MosekSolver::is_available() { return false; }

void MosekSolver::DoSolve(
    const MathematicalProgram&, const Eigen::VectorXd&,
    const SolverOptions&, MathematicalProgramResult*) const {
  throw runtime_error(
      "Mosek is not installed in your build. You'll need to use a different "
      "solver.");
}

}  // namespace solvers
}  // namespace drake
