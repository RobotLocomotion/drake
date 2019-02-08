#include "drake/solvers/choose_best_solver.h"

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

std::unique_ptr<SolverInterface> MakeSolver(const SolverId& id) {
  if (id == LinearSystemSolver::id()) {
    return std::make_unique<LinearSystemSolver>();
  } else if (id == EqualityConstrainedQPSolver::id()) {
    return std::make_unique<EqualityConstrainedQPSolver>();
  } else if (id == MosekSolver::id()) {
    return std::make_unique<MosekSolver>();
  } else if (id == GurobiSolver::id()) {
    return std::make_unique<GurobiSolver>();
  } else if (id == OsqpSolver::id()) {
    return std::make_unique<OsqpSolver>();
  } else if (id == MobyLcpSolverId::id()) {
    return std::make_unique<MobyLCPSolver<double>>();
  } else if (id == SnoptSolver::id()) {
    return std::make_unique<SnoptSolver>();
  } else if (id == IpoptSolver::id()) {
    return std::make_unique<IpoptSolver>();
  } else if (id == NloptSolver::id()) {
    return std::make_unique<NloptSolver>();
  } else if (id == ScsSolver::id()) {
    return std::make_unique<ScsSolver>();
  } else {
    throw std::invalid_argument("MakeSolver: no matching solver " + id.name());
  }
}
}  // namespace solvers
}  // namespace drake
