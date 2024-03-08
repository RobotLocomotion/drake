#include "drake/solvers/solver_base.h"

#include <limits>
#include <utility>

#include <fmt/format.h>

#include "drake/common/drake_assert.h"
#include "drake/common/nice_type_name.h"

namespace drake {
namespace solvers {

SolverBase::SolverBase(
    const SolverId& id, std::function<bool()> available,
    std::function<bool()> enabled,
    std::function<bool(const MathematicalProgram&)> are_satisfied,
    std::function<std::string(const MathematicalProgram&)> explain_unsatisfied)
    : solver_id_(id),
      default_available_(std::move(available)),
      default_enabled_(std::move(enabled)),
      default_are_satisfied_(std::move(are_satisfied)),
      default_explain_unsatisfied_(std::move(explain_unsatisfied)) {}

SolverBase::~SolverBase() = default;

MathematicalProgramResult SolverBase::Solve(
    const MathematicalProgram& prog,
    const std::optional<Eigen::VectorXd>& initial_guess,
    const std::optional<SolverOptions>& solver_options) const {
  MathematicalProgramResult result;
  this->Solve(prog, initial_guess, solver_options, &result);
  return result;
}

namespace {
std::string ShortName(const SolverInterface& solver) {
  return NiceTypeName::RemoveNamespaces(NiceTypeName::Get(solver));
}

template <typename T>
T GetOptionOrDefault(
    SolverId solver_id,
    const std::optional<SolverOptions>& merged_options,
    const std::string& key,
    T default_value) {
  if (!merged_options) {
    return default_value;
  }
  const auto& options = merged_options->GetOptions<T>(solver_id);
  auto iter = options.find(key);
  if (iter != options.end()) {
    return iter->second;
  } else {
    return default_value;
  }
}
}  // namespace

void SolverBase::Solve(const MathematicalProgram& prog,
                       const std::optional<Eigen::VectorXd>& initial_guess,
                       const std::optional<SolverOptions>& solver_options,
                       MathematicalProgramResult* result) const {
  const bool warm_start_dual = GetOptionOrDefault<int>(
      solver_id(), solver_options, "drake_warm_start_dual", 0);
  if (!warm_start_dual) {
    // If we are warm starting via dual, we will rely on the solver itself to
    // keep relevant data from the the prior result.
    // Note: Solvers that are warm-starting should take care to retrieve the
    // results they need and then clear out prior results.
    *result = {};
  }

  if (!available()) {
    const std::string name = ShortName(*this);
    throw std::invalid_argument(fmt::format(
        "{} cannot Solve because {}::available() is false, i.e.,"
        " {} has not been compiled as part of this binary."
        " Refer to the {} class overview documentation for how to compile it.",
        name, name, name, name));
  }
  if (!enabled()) {
    const std::string name = ShortName(*this);
    throw std::invalid_argument(fmt::format(
        "{} cannot Solve because {}::enabled() is false, i.e.,"
        " {} has not been properly configured for use."
        " Typically this means that an environment variable has not been set."
        " Refer to the {} class overview documentation for how to enable it.",
        name, name, name, name));
  }
  if (!AreProgramAttributesSatisfied(prog)) {
    throw std::invalid_argument(ExplainUnsatisfiedProgramAttributes(prog));
  }
  result->set_solver_id(solver_id());
  result->set_decision_variable_index(prog.decision_variable_index());
  const Eigen::VectorXd& x_init =
      initial_guess ? *initial_guess : prog.initial_guess();
  if (x_init.rows() != prog.num_vars()) {
    throw std::invalid_argument(
        fmt::format("Solve expects initial guess of size {}, got {}.",
                    prog.num_vars(), x_init.rows()));
  }
  if (!solver_options) {
    DoSolve(prog, x_init, prog.solver_options(), result);
  } else {
    SolverOptions merged_options = *solver_options;
    merged_options.Merge(prog.solver_options());
    DoSolve(prog, x_init, merged_options, result);
  }
}

bool SolverBase::available() const {
  DRAKE_DEMAND(default_available_ != nullptr);
  return default_available_();
}

bool SolverBase::enabled() const {
  DRAKE_DEMAND(default_enabled_ != nullptr);
  return default_enabled_();
}

bool SolverBase::AreProgramAttributesSatisfied(
    const MathematicalProgram& prog) const {
  DRAKE_DEMAND(default_are_satisfied_ != nullptr);
  return default_are_satisfied_(prog);
}

std::string SolverBase::ExplainUnsatisfiedProgramAttributes(
    const MathematicalProgram& prog) const {
  if (default_explain_unsatisfied_ != nullptr) {
    return default_explain_unsatisfied_(prog);
  }
  if (AreProgramAttributesSatisfied(prog)) {
    return {};
  }
  return fmt::format("{} is unable to solve a MathematicalProgram with {}.",
                     ShortName(*this), to_string(prog.required_capabilities()));
}

}  // namespace solvers
}  // namespace drake
