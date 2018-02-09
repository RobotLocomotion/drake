#include "drake/solvers/osqp_solver.h"

#include "osqp.h"

namespace drake {
namespace solvers {
bool OsqpSolver::available() const { return true; }

SolutionResult OsqpSolver::Solve(MathematicalProgram& prog) const {
  // OSQP solves a convex quadratic programming problem
  // min 0.5 xᵀPx + qᵀx
  // s.t l ≤ Ax ≤ u
  // OSQP is written in C, so this function will be in C style.
  
  // Problem settings
  OSQPSettings* settings = static_cast<OSQPSettings*>(c_malloc(sizeof(OSQPSettings)));

  // OSQP structures
  OSQPWorkspace* work;  // Workspace
  OSQPData* data;  // OSQPData
  
  // Populate data
  data = static_cast<OSQPData*>(c_malloc(sizeof(OSQPData)));


  return SolutionResult::kSolutionFound;
}
}  // namespace solvers
}  // namespace drake
