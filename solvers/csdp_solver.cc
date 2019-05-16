#include "drake/solvers/csdp_solver.h"

#include <algorithm>
#include <limits>
#include <unordered_map>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Sparse>
#include <Eigen/SparseQR>

namespace drake {
namespace solvers {

void CsdpSolver::DoSolve(const MathematicalProgram&, const Eigen::VectorXd&,
                         const SolverOptions&,
                         MathematicalProgramResult*) const {
  throw std::runtime_error("CsdpSolver::DoSolve() not supported yet");
}

bool CsdpSolver::is_available() { return true; }
}  // namespace solvers
}  // namespace drake
