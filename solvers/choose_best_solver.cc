#include "drake/solvers/choose_best_solver.h"

#include <array>
#include <string>
#include <unordered_map>

#include "drake/common/drake_assert.h"
#include "drake/common/never_destroyed.h"
#include "drake/solvers/clp_solver.h"
#include "drake/solvers/csdp_solver.h"
#include "drake/solvers/equality_constrained_qp_solver.h"
#include "drake/solvers/get_program_type.h"
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

template <typename SomeSolver, typename... SomeSolvers>
SolverId ChooseFirstMatchingSolver(
    const MathematicalProgram& prog,
    std::string_view additional_error_message = "") {
  if (IsMatch<SomeSolver>(prog)) {
    return SomeSolver::id();
  } else {
    if constexpr (sizeof...(SomeSolvers) > 0) {
      return ChooseFirstMatchingSolver<SomeSolvers...>(
          prog, additional_error_message);
    } else {
      throw std::invalid_argument(fmt::format(
          "There is no available solver for the optimization program{}",
          additional_error_message));
    }
  }
}

}  // namespace

SolverId ChooseBestSolver(const MathematicalProgram& prog) {
  const ProgramType program_type = GetProgramType(prog);
  switch (program_type) {
    case ProgramType::kLP: {
      return ChooseFirstMatchingSolver<
          // Preferred solvers.
          // LinearSystemSolver solves a specific type of problem A*x=b, we put
          // the more specific solver ahead of other general solvers.
          // The order between Mosek and Gurobi can be changed. According to
          // the benchmark http://plato.asu.edu/bench.html, Gurobi is slightly
          // faster than Mosek (as of 07-26-2021), but the difference is not
          // significant.
          // Clp is slower than Gurobi and Mosek (with the barrier method).
          LinearSystemSolver, GurobiSolver, MosekSolver, ClpSolver,
          // Dispreferred (generic nonlinear solvers).
          // I generally find SNOPT faster than IPOPT. Nlopt is less reliable.
          SnoptSolver, IpoptSolver, NloptSolver,
          // Dispreferred (cannot handle free variables).
          CsdpSolver,
          // Dispreferred (ADMM, low accuracy).
          ScsSolver>(prog);
    }
    case ProgramType::kQP: {
      return ChooseFirstMatchingSolver<
          // Preferred solvers.
          // EqualityConstrainedQPSolver solves a specific QP with only linear
          // equality constraints. We put the more specific solver ahead of the
          // general solvers.
          // According to http://plato.asu.edu/ftp/cconvex.html, Mosek is
          // slightly faster than Gurobi. In practice, I find their performance
          // comparable, so the order between these two can be switched.
          EqualityConstrainedQPSolver, MosekSolver, GurobiSolver,
          // Dispreferred (ADMM, low accuracy).
          // Although both OSQP and SCS use ADMM, I find OSQP to be more
          // accurate than SCS. Oftentime I find OSQP generates solution with
          // reasonable accuracy, so I put it before SNOPT/IPOPT/NLOPT (which
          // are often slower than OSQP).
          OsqpSolver,
          // TODO(hongkai.dai): add CLP to this list when we resolve the memory
          // issue in CLP. Dispreferred (generic nonlinear solvers). I find
          // SNOPT often faster than IPOPT. NLOPT is less reliable.
          SnoptSolver, IpoptSolver, NloptSolver,
          // Dispreferred (ADMM, low accuracy).
          ScsSolver>(prog);
    }
    case ProgramType::kSOCP: {
      return ChooseFirstMatchingSolver<
          // Preferred solvers.
          // According to http://plato.asu.edu/ftp/socp.html, Mosek is slightly
          // faster than Gurobi for SOCP, but the difference is small.
          MosekSolver, GurobiSolver,
          // Dispreferred (cannot handle free variables).
          CsdpSolver,
          // Dispreferred (ADMM, low accuracy).
          ScsSolver,
          // Dispreferred (generic nonlinear solvers).
          // I strongly suggest NOT to use these solvers for SOCP. These
          // gradient-based solvers ignore the convexity of the problem, and
          // also these solvers require smooth gradient, while the second-order
          // cone constraint is not differentiable at the tip of the cone.
          SnoptSolver, IpoptSolver, NloptSolver>(prog);
    }
    case ProgramType::kSDP: {
      return ChooseFirstMatchingSolver<
          // Preferred solvers.
          MosekSolver,
          // Dispreferred (cannot handle free variables).
          CsdpSolver,
          // Dispreferred (ADMM, low accuracy).
          ScsSolver,
          // Dispreferred (generic nonlinear solvers).
          // I strongly suggest NOT to use these solvers for SDP. These
          // gradient-based solvers ignore the convexity of the problem, and
          // also these solvers require smooth gradient, while the semidefinite
          // cone constraint is not differentiable everywhere (when the minimal
          // eigen value is not unique).
          SnoptSolver, IpoptSolver, NloptSolver>(prog);
    }
    case ProgramType::kGP:
    case ProgramType::kCGP: {
      // TODO(hongkai.dai): These types of programs should not be called GP or
      // CGP. GP are programs with posynomial constraints. Find the right name
      // of these programs with exponential cones, and add to the documentation
      // in https://drake.mit.edu/doxygen_cxx/group__solvers.html
      return ChooseFirstMatchingSolver<
          // Preferred solver.
          MosekSolver,
          // Dispreferred solver, low accuracy (with ADMM method).
          ScsSolver>(prog);
    }
    case ProgramType::kMILP:
    case ProgramType::kMIQP:
    case ProgramType::kMISOCP: {
      // According to http://plato.asu.edu/ftp/misocp.html, Gurobi is a lot
      // faster than Mosek for MISOCP. The benchmark doesn't compare Mosek
      // against Gurobi for MILP and MIQP.
      return ChooseFirstMatchingSolver<GurobiSolver, MosekSolver>(
          prog,
          ", please manually instantiate MixedIntegerBranchAndBound and solve "
          "the problem if the problem size is small, typically with less than "
          "a dozen of binary variables.");
    }
    case ProgramType::kMISDP: {
      throw std::runtime_error(
          "ChooseBestSolver():The MISDP problem is not well-supported yet. You "
          "can try Drake's implementation MixedIntegerBranchAndBound for small "
          "sized MISDP.");
    }
    case ProgramType::kQuadraticCostConicConstraint: {
      return ChooseFirstMatchingSolver<
          // Preferred solvers.
          // I don't know if Mosek is better than Gurobi for this type of
          // programs.
          MosekSolver, GurobiSolver,
          // Dispreferred solver (ADMM, low accuracy).
          ScsSolver>(prog);
    }
    case ProgramType::kNLP: {
      // I find SNOPT often faster than IPOPT. NLOPT is less reliable.
      return ChooseFirstMatchingSolver<SnoptSolver, IpoptSolver, NloptSolver>(
          prog);
    }
    case ProgramType::kLCP: {
      return ChooseFirstMatchingSolver<
          // Preferred solver.
          MobyLCPSolver<double>,
          // Dispreferred solver (generic nonlinear solver).
          SnoptSolver, IpoptSolver, NloptSolver>(prog);
    }
    case ProgramType::kUnknown: {
      // The order of solvers listed here is an approximation of a total order,
      // drawn from all of the partial orders given throughout the other case
      // statements shown above.
      return ChooseFirstMatchingSolver<
          LinearSystemSolver, EqualityConstrainedQPSolver, MosekSolver,
          GurobiSolver, OsqpSolver, ClpSolver, MobyLCPSolver<double>,
          SnoptSolver, IpoptSolver, NloptSolver, CsdpSolver, ScsSolver>(prog);
    }
  }
  DRAKE_UNREACHABLE();
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
  for (const auto& solver_id : solver_ids) {
    solver_names.append(solver_id.name() + " ");
  }
  throw std::runtime_error("MakeFirstAvailableSolver(): none of the solvers " +
                           solver_names + "is available and enabled.");
}

}  // namespace solvers
}  // namespace drake
