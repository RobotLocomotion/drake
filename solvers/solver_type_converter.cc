#include "drake/solvers/solver_type_converter.h"

#include "drake/common/drake_assert.h"
#include "drake/solvers/clp_solver.h"
#include "drake/solvers/csdp_solver.h"
#include "drake/solvers/dreal_solver.h"
#include "drake/solvers/equality_constrained_qp_solver.h"
#include "drake/solvers/gurobi_solver.h"
#include "drake/solvers/ibex_solver.h"
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

SolverId SolverTypeConverter::TypeToId(SolverType solver_type) {
  switch (solver_type) {
    case SolverType::kClp:
      return ClpSolver::id();
    case SolverType::kCsdp:
      return CsdpSolver::id();
    case SolverType::kDReal:
      return DrealSolver::id();
    case SolverType::kEqualityConstrainedQP:
      return EqualityConstrainedQPSolver::id();
    case SolverType::kGurobi:
      return GurobiSolver::id();
    case SolverType::kIbex:
      return IbexSolver::id();
    case SolverType::kIpopt:
      return IpoptSolver::id();
    case SolverType::kLinearSystem:
      return LinearSystemSolver::id();
    case SolverType::kMobyLCP:
      return MobyLcpSolverId::id();
    case SolverType::kMosek:
      return MosekSolver::id();
    case SolverType::kNlopt:
      return NloptSolver::id();
    case SolverType::kOsqp:
      return OsqpSolver::id();
    case SolverType::kSnopt:
      return SnoptSolver::id();
    case SolverType::kScs:
      return ScsSolver::id();
    case SolverType::kUnrevisedLemke:
      return UnrevisedLemkeSolverId::id();
  }
  DRAKE_UNREACHABLE();
}

std::optional<SolverType> SolverTypeConverter::IdToType(SolverId solver_id) {
  if (solver_id == ClpSolver::id()) {
    return SolverType::kClp;
  } else if (solver_id == CsdpSolver::id()) {
    return SolverType::kCsdp;
  } else if (solver_id == DrealSolver::id()) {
    return SolverType::kDReal;
  } else if (solver_id == EqualityConstrainedQPSolver::id()) {
    return SolverType::kEqualityConstrainedQP;
  } else if (solver_id == GurobiSolver::id()) {
    return SolverType::kGurobi;
  } else if (solver_id == IbexSolver::id()) {
    return SolverType::kIbex;
  } else if (solver_id == IpoptSolver::id()) {
    return SolverType::kIpopt;
  } else if (solver_id == LinearSystemSolver::id()) {
    return SolverType::kLinearSystem;
  } else if (solver_id == MobyLcpSolverId::id()) {
    return SolverType::kMobyLCP;
  } else if (solver_id == MosekSolver::id()) {
    return SolverType::kMosek;
  } else if (solver_id == NloptSolver::id()) {
    return SolverType::kNlopt;
  } else if (solver_id == SnoptSolver::id()) {
    return SolverType::kSnopt;
  } else if (solver_id == OsqpSolver::id()) {
    return SolverType::kOsqp;
  } else if (solver_id == ScsSolver::id()) {
    return SolverType::kScs;
  } else if (solver_id == UnrevisedLemkeSolverId::id()) {
    return SolverType::kUnrevisedLemke;
  } else {
    return std::nullopt;
  }
}

}  // namespace solvers
}  // namespace drake
