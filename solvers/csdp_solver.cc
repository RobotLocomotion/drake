#include "drake/solvers/csdp_solver.h"

#include <stdexcept>

#include "drake/solvers/csdp_solver_internal.h"

namespace drake {
namespace solvers {

void CsdpSolver::DoSolve(const MathematicalProgram&, const Eigen::VectorXd&,
                         const SolverOptions&,
                         MathematicalProgramResult*) const {
  // TODO(hongkai.dai): support DoSolve in subsequent PRs.
  throw std::runtime_error("CsdpSolver::DoSolve() not supported yet");
}

bool CsdpSolver::is_available() { return false; }
}  // namespace solvers
}  // namespace drake
