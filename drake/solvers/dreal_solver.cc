#include "drake/solvers/dreal_solver.h"

#include <stdexcept>
#include "dreal/dreal.h"

#include "drake/solvers/mathematical_program.h"

namespace drake {
namespace solvers {
bool DrealSolver::available_impl() const {
  // It returns false for now, because nothing is implemented yet.
  return false;
}

DrealSolverResult* DrealSolver::Solve_impl(
    MathematicalProgram* const prog) const {
  throw std::runtime_error("not implemented yet");
}
}  // namespace solvers
}  // namespace drake
