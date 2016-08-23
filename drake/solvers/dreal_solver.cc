#include "drake/solvers/dreal_solver.h"

#include <stdexcept>
#include "dreal/dreal.h"

#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/optimization.h"

namespace drake {
namespace solvers {
bool DrealSolver::available() const {
  // It returns false for now, because nothing is implemented yet.
  return false;
}

SolutionResult DrealSolver::Solve(OptimizationProblem& prog) const {
  throw std::runtime_error("not implemented yet");
}
}  // namespace solvers
}  // namespace drake
