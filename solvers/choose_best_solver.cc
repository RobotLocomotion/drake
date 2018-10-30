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
#include "drake/solvers/unrevised_lemke_solver.h"

namespace drake {
namespace solvers {
std::unique_ptr<MathematicalProgramSolverInterface> ChooseBestSolver(
    const MathematicalProgram& prog) {
  if (LinearSystemSolver::IsAvailable() &&
      LinearSystemSolver::ProgramAttributesSatisfied(prog)) {
    return std::make_unique<LinearSystemSolver>();
  } else if (EqualityConstrainedQPSolver::IsAvailable() &&
             EqualityConstrainedQPSolver::ProgramAttributesSatisfied(prog)) {
    return std::make_unique<EqualityConstrainedQPSolver>();
  } else if (MosekSolver::IsAvailable() &&
             MosekSolver::ProgramAttributesSatisfied(prog)) {
    // TODO(hongkai.dai@tri.global): based on my limited experience, Mosek is
    // faster than Gurobi for convex optimization problem. But we should run
    // a more thorough comparison.
    return std::make_unique<MosekSolver>();
  } else if (GurobiSolver::IsAvailable() &&
             GurobiSolver::ProgramAttributesSatisfied(prog)) {
    return std::make_unique<GurobiSolver>();
  } else if (OsqpSolver::IsAvailable() &&
             OsqpSolver::ProgramAttributesSatisfied(prog)) {
    return std::make_unique<OsqpSolver>();
  } else if (MobyLCPSolver<double>::IsAvailable() &&
             MobyLCPSolver<double>::ProgramAttributesSatisfied(prog)) {
    return std::make_unique<MobyLCPSolver<double>>();
  } else if (SnoptSolver::IsAvailable() &&
             SnoptSolver::ProgramAttributesSatisfied(prog)) {
    return std::make_unique<SnoptSolver>();
  } else if (IpoptSolver::IsAvailable() &&
             IpoptSolver::ProgramAttributesSatisfied(prog)) {
    return std::make_unique<IpoptSolver>();
  } else if (NloptSolver::IsAvailable() &&
             NloptSolver::ProgramAttributesSatisfied(prog)) {
    return std::make_unique<NloptSolver>();
  } else if (ScsSolver::IsAvailable() &&
             ScsSolver::ProgramAttributesSatisfied(prog)) {
    // Use SCS as the last resort. SCS uses ADMM method, which converges fast to
    // modest accuracy quite fast, but then slows down significantly if the user
    // wants high accuracy.
    return std::make_unique<ScsSolver>();
  }

  throw std::invalid_argument(
      "There is no available solver for the optimization program");
}
}  // namespace solvers
}  // namespace drake
