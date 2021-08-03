#include "drake/solvers/choose_best_solver.h"

#include <array>
#include <string>
#include <unordered_map>

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

// A collection of function pointers to the static member functions that are
// twins to the SolverInterface virtual member functions.  These pointers are
// useful when we want to interrogate solvers without constructing them.
class StaticSolverInterface {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(StaticSolverInterface)

  template <typename SomeSolver>
  static constexpr StaticSolverInterface Make() {
    StaticSolverInterface result;
    result.id_ = &SomeSolver::id;
    result.is_available_ = &SomeSolver::is_available;
    result.is_enabled_ = &SomeSolver::is_enabled;
    result.make_ = &MakeAndUpcast<SomeSolver>;
    return result;
  }

  SolverId id() const { return (*id_)(); }
  bool is_available() const { return (*is_available_)(); }
  bool is_enabled() const { return (*is_enabled_)(); }
  std::unique_ptr<SolverInterface> Make() const { return (*make_)(); }

 private:
  StaticSolverInterface() = default;

  // A helper function that combines make_unique with an upcast.
  template <typename SomeSolver>
  static std::unique_ptr<SolverInterface> MakeAndUpcast() {
    return std::make_unique<SomeSolver>();
  }

  SolverId(*id_)() = nullptr;
  bool(*is_available_)() = nullptr;
  bool(*is_enabled_)() = nullptr;
  std::unique_ptr<SolverInterface>(*make_)() = nullptr;
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

// The list of all solvers compiled in Drake, indexed by SolverId.
using KnownSolversMap = std::unordered_map<
    SolverId, const StaticSolverInterface*>;
const KnownSolversMap& GetKnownSolversMap() {
  static const never_destroyed<KnownSolversMap> result{[]() {
    KnownSolversMap prototype;
    for (const auto& solver : kKnownSolvers) {
      prototype.emplace(solver.id(), &solver);
    }
    return prototype;
  }()};
  return result.access();
}

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
    std::set<SolverId> prototype;
    for (const auto& solver : kKnownSolvers) {
      prototype.insert(solver.id());
    }
    return prototype;
  }()};
  return result.access();
}

std::unique_ptr<SolverInterface> MakeSolver(const SolverId& id) {
  const auto& map = GetKnownSolversMap();
  auto iter = map.find(id);
  if (iter != map.end()) {
    const StaticSolverInterface& solver = *(iter->second);
    return solver.Make();
  }
  throw std::invalid_argument("MakeSolver: no matching solver " + id.name());
}

std::unique_ptr<SolverInterface> MakeFirstAvailableSolver(
    const std::vector<SolverId>& solver_ids) {
  const auto& map = GetKnownSolversMap();
  for (const auto& solver_id : solver_ids) {
    auto iter = map.find(solver_id);
    if (iter != map.end()) {
      const StaticSolverInterface& solver = *(iter->second);
      if (solver.is_available() && solver.is_enabled()) {
        return solver.Make();
      }
    }
  }
  std::string solver_names = "";
  for (const auto solver_id : solver_ids) {
    solver_names.append(solver_id.name() + " ");
  }
  throw std::runtime_error("MakeFirstAvailableSolver(): none of the solvers " +
                           solver_names + "is available and enabled.");
}

}  // namespace solvers
}  // namespace drake
