/* clang-format off to disable clang-format-includes */
#include "drake/solvers/dreal_solver.h"
/* clang-format on */

#include <stdexcept>

namespace drake {
namespace solvers {

std::optional<DrealSolver::IntervalBox> DrealSolver::CheckSatisfiability(
    const symbolic::Formula&, double) {
  throw std::runtime_error(
      "The dReal bindings were not compiled.  You'll need to use a different "
      "solver.");
}

std::optional<DrealSolver::IntervalBox> DrealSolver::Minimize(
    const symbolic::Expression&,
    const symbolic::Formula&,
    double,
    DrealSolver::LocalOptimization) {
  throw std::runtime_error(
      "The dReal bindings were not compiled.  You'll need to use a different "
      "solver.");
}

bool DrealSolver::is_available() { return false; }

void DrealSolver::DoSolve(
    const MathematicalProgram&,
    const Eigen::VectorXd&,
    const SolverOptions&,
    MathematicalProgramResult*) const {
  throw std::runtime_error(
      "The dReal bindings were not compiled.  You'll need to use a different "
      "solver.");
}

}  // namespace solvers
}  // namespace drake
