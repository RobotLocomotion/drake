#include "drake/solvers/choose_best_solver.h"

#include <array>

#include "drake/common/never_destroyed.h"
#include "drake/solvers/clp_solver.h"
#include "drake/solvers/csdp_solver.h"
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
namespace {

// A helper function that combines make_unique with an upcast.
template <typename SomeSolver>
std::unique_ptr<SolverInterface> MakeSolverInterface() {
  return std::make_unique<SomeSolver>();
}

// A collection of function pointers to the static member functions that are
// twins to the SolverInterface virtual member functions.  These pointers are
// useful when we want to interrogate solvers without constructing them.
struct StaticSolverInterface {
  template <typename SomeSolver>
  static constexpr StaticSolverInterface Make() {
    return {
      .id = &SomeSolver::id,
      .is_available = &SomeSolver::is_available,
      .is_enabled = &SomeSolver::is_enabled,
      .is_satisfied = &SomeSolver::ProgramAttributesSatisfied,
      .make = &MakeSolverInterface<SomeSolver>,
    };
  }

  SolverId(*id)() = nullptr;
  bool(*is_available)() = nullptr;
  bool(*is_enabled)() = nullptr;
  bool(*is_satisfied)(const MathematicalProgram&) = nullptr;
  std::unique_ptr<SolverInterface>(*make)() = nullptr;
};

// The list of all solvers compiled in Drake.
constexpr std::array<StaticSolverInterface, 12> kKnownSolvers{
    StaticSolverInterface::Make<ClpSolver>(),
    StaticSolverInterface::Make<CsdpSolver>(),
    StaticSolverInterface::Make<EqualityConstrainedQPSolver>(),
    StaticSolverInterface::Make<GurobiSolver>(),
    StaticSolverInterface::Make<IpoptSolver>(),
    StaticSolverInterface::Make<LinearSystemSolver>(),
    StaticSolverInterface::Make<MobyLCPSolver<double>>(),
    StaticSolverInterface::Make<MosekSolver>(),
    StaticSolverInterface::Make<NloptSolver>(),
    StaticSolverInterface::Make<OsqpSolver>(),
    StaticSolverInterface::Make<ScsSolver>(),
    StaticSolverInterface::Make<SnoptSolver>()};

template <typename SomeSolver>
bool IsMatch(const MathematicalProgram& prog) {
  return SomeSolver::is_available() && SomeSolver::is_enabled() &&
      SomeSolver::ProgramAttributesSatisfied(prog);
}

}  // namespace

SolverId ChooseBestSolver(const MathematicalProgram& prog) {
  if (IsMatch<LinearSystemSolver>(prog)) {
    return LinearSystemSolver::id();
  } else if (IsMatch<EqualityConstrainedQPSolver>(prog)) {
    return EqualityConstrainedQPSolver::id();
  } else if (IsMatch<MosekSolver>(prog)) {
    // TODO(hongkai.dai@tri.global): based on my limited experience, Mosek is
    // faster than Gurobi for convex optimization problem. But we should run
    // a more thorough comparison.
    return MosekSolver::id();
  } else if (IsMatch<GurobiSolver>(prog)) {
    return GurobiSolver::id();
  } else if (IsMatch<OsqpSolver>(prog)) {
    // For a QP, we prioritize OSQP over CLP.
    // For an LP, we prioritize CLP, and don't allow solving LP with OSQP.
    return OsqpSolver::id();
  } else if (IsMatch<ClpSolver>(prog)) {
    return ClpSolver::id();
  } else if (IsMatch<MobyLCPSolver<double>>(prog)) {
    return MobyLcpSolverId::id();
  } else if (IsMatch<SnoptSolver>(prog)) {
    return SnoptSolver::id();
  } else if (IsMatch<IpoptSolver>(prog)) {
    return IpoptSolver::id();
  } else if (IsMatch<NloptSolver>(prog)) {
    return NloptSolver::id();
  } else if (IsMatch<CsdpSolver>(prog)) {
    return CsdpSolver::id();
  } else if (IsMatch<ScsSolver>(prog)) {
    // Use SCS as the last resort. SCS uses ADMM method, which converges fast to
    // modest accuracy quite fast, but then slows down significantly if the user
    // wants high accuracy.
    return ScsSolver::id();
  }
  throw std::invalid_argument(
      "There is no available solver for the optimization program");
}

const std::set<SolverId>& GetKnownSolvers() {
  static const never_destroyed<std::set<SolverId>> result{[]() {
    std::set<SolverId> accumulator;
    for (const auto& known_solver : kKnownSolvers) {
      const SolverId id = (*known_solver.id)();
      accumulator.insert(id);
    }
    return accumulator;
  }()};
  return result.access();
}

std::unique_ptr<SolverInterface> MakeSolver(const SolverId& id) {
  for (const auto& known_solver : kKnownSolvers) {
    const SolverId known_id = (*known_solver.id)();
    if (id == known_id) {
      return (*known_solver.make)();
    }
  }
  throw std::invalid_argument("MakeSolver: no matching solver " + id.name());
}

}  // namespace solvers
}  // namespace drake
