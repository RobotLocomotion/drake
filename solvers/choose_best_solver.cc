#include "drake/solvers/choose_best_solver.h"

#include "drake/solvers/dreal_solver.h"
#include "drake/solvers/equality_constrained_qp_solver.h"
#include "drake/solvers/gurobi_solver.h"
#include "drake/solvers/ipopt_solver.h"
#include "drake/solvers/linear_system_solver.h"
#include "drake/solvers/moby_lcp_solver.h"
#include "drake/solvers/mosek_solver.h"
#include "drake/solvers/nlopt_solver.h"
#include "drake/solvers/osqp_solver.h"
#include "drake/solvers/scs_solver.h"
#include "drake/solvers/snopt_solver.h"

namespace drake {
namespace solvers {
SolverId ChooseBestSolver(const MathematicalProgram& prog) {
  if (LinearSystemSolver::is_available() &&
      LinearSystemSolver::ProgramAttributesSatisfied(prog)) {
    return LinearSystemSolver::id();
  } else if (EqualityConstrainedQPSolver::is_available() &&
             EqualityConstrainedQPSolver::ProgramAttributesSatisfied(prog)) {
    return EqualityConstrainedQPSolver::id();
  } else if (MosekSolver::is_available() &&
             MosekSolver::ProgramAttributesSatisfied(prog)) {
    // TODO(hongkai.dai@tri.global): based on my limited experience, Mosek is
    // faster than Gurobi for convex optimization problem. But we should run
    // a more thorough comparison.
    return MosekSolver::id();
  } else if (GurobiSolver::is_available() &&
             GurobiSolver::ProgramAttributesSatisfied(prog)) {
    return GurobiSolver::id();
  } else if (OsqpSolver::is_available() &&
             OsqpSolver::ProgramAttributesSatisfied(prog)) {
    return OsqpSolver::id();
  } else if (MobyLCPSolver<double>::is_available() &&
             MobyLCPSolver<double>::ProgramAttributesSatisfied(prog)) {
    return MobyLcpSolverId::id();
  } else if (SnoptSolver::is_available() &&
             SnoptSolver::ProgramAttributesSatisfied(prog)) {
    return SnoptSolver::id();
  } else if (IpoptSolver::is_available() &&
             IpoptSolver::ProgramAttributesSatisfied(prog)) {
    return IpoptSolver::id();
  } else if (NloptSolver::is_available() &&
             NloptSolver::ProgramAttributesSatisfied(prog)) {
    return NloptSolver::id();
  } else if (ScsSolver::is_available() &&
             ScsSolver::ProgramAttributesSatisfied(prog)) {
    // Use SCS as the last resort. SCS uses ADMM method, which converges fast to
    // modest accuracy quite fast, but then slows down significantly if the user
    // wants high accuracy.
    return ScsSolver::id();
  }

  throw std::invalid_argument(
      "There is no available solver for the optimization program");
}
}  // namespace solvers
}  // namespace drake
