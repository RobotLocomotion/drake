#include "drake/solvers/solver_base.h"

#include <utility>

#include <fmt/format.h>

#include "drake/common/drake_assert.h"
#include "drake/common/nice_type_name.h"

namespace drake {
namespace solvers {

SolverBase::SolverBase(
    std::function<SolverId()> id,
    std::function<bool()> available,
    std::function<bool(const MathematicalProgram&)> satisfied)
    : default_id_(std::move(id)),
      default_available_(std::move(available)),
      default_satisfied_(std::move(satisfied)) {}

SolverBase::~SolverBase() = default;

MathematicalProgramResult SolverBase::Solve(
    const MathematicalProgram& prog,
    const optional<Eigen::VectorXd>& initial_guess,
    const optional<SolverOptions>& solver_options) const {
  MathematicalProgramResult result;
  this->Solve(prog, initial_guess, solver_options, &result);
  return result;
}

void SolverBase::Solve(const MathematicalProgram& prog,
                       const optional<Eigen::VectorXd>& initial_guess,
                       const optional<SolverOptions>& solver_options,
                       MathematicalProgramResult* result) const {
  *result = {};
  if (!available()) {
    throw std::invalid_argument(fmt::format(
        "The {} is not available in this build", NiceTypeName::Get(*this)));
  }
  if (!AreProgramAttributesSatisfied(prog)) {
    throw std::invalid_argument(fmt::format(
        "The capabilities of {} do not meet the requirements of the "
        "MathematicalProgram ({})", NiceTypeName::Get(*this),
        to_string(prog.required_capabilities())));
  }
  result->set_solver_id(solver_id());
  result->set_decision_variable_index(prog.decision_variable_index());
  const Eigen::VectorXd& x_init =
      initial_guess ? *initial_guess : prog.initial_guess();
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

SolverId SolverBase::solver_id() const {
  DRAKE_DEMAND(default_id_ != nullptr);
  return default_id_();
}

bool SolverBase::AreProgramAttributesSatisfied(
    const MathematicalProgram& prog) const {
  DRAKE_DEMAND(default_satisfied_ != nullptr);
  return default_satisfied_(prog);
}

// NOLINTNEXTLINE(runtime/references)
SolutionResult SolverBase::Solve(MathematicalProgram& prog) const {
  MathematicalProgramResult result;
  Solve(prog, {}, {}, &result);
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
  prog.SetSolverResult(result.ConvertToSolverResult());
#pragma GCC diagnostic pop
  return result.get_solution_result();
}

}  // namespace solvers
}  // namespace drake
