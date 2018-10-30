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
  if (LinearSystemSolver::available() &&
      LinearSystemSolver::ProgramAttributesSatisfied(prog)) {
    return std::make_unique<LinearSystemSolver>();
  } else if (EqualityConstrainedQPSolver::available() &&
             EqualityConstrainedQPSolver::ProgramAttributesSatisfied(prog)) {
    return std::make_unique<EqualityConstrainedQPSolver>();
  } else if (MosekSolver::available() &&
             MosekSolver::ProgramAttributesSatisfied(prog)) {
    return std::make_unique<MosekSolver>();
  } else if (GurobiSolver::available() &&
             GurobiSolver::ProgramAttributesSatisfied(prog)) {
    return std::make_unique<GurobiSolver>();
  } else if (OsqpSolver::available() &&
             OsqpSolver::ProgramAttributesSatisfied(prog)) {
    return std::make_unique<OsqpSolver>();
  } else if (MobyLCPSolver<double>::available() &&
             MobyLCPSolver<double>::ProgramAttributesSatisfied(prog)) {
    return std::make_unique<MobyLCPSolver<double>>();
  } else if (UnrevisedLemkeSolver<double>::available() &&
             UnrevisedLemkeSolver<double>::ProgramAttributesSatisfied(prog)) {
    return std::make_unique<UnrevisedLemkeSolver<double>>();
  } else if (SnoptSolver::available() &&
             SnoptSolver::ProgramAttributesSatisfied(prog)) {
    return std::make_unique<SnoptSolver>();
  } else if (IpoptSolver::available() &&
             IpoptSolver::ProgramAttributesSatisfied(prog)) {
    return std::make_unique<IpoptSolver>();
  } else if (NloptSolver::available() &&
             NloptSolver::ProgramAttributesSatisfied(prog)) {
    return std::make_unique<NloptSolver>();
  } else if (ScsSolver::available() &&
             ScsSolver::ProgramAttributesSatisfied(prog)) {
    return std::make_unique<ScsSolver>();
  }

  throw std::invalid_argument(
      "There is no available solver for the optimization program");
}
}  // namespace solvers
}  // namespace drake
