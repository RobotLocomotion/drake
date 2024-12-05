/* clang-format off to disable clang-format-includes */
#include "drake/solvers/ipopt_solver.h"
/* clang-format on */

#include "drake/common/never_destroyed.h"
#include "drake/solvers/mathematical_program.h"

namespace drake {
namespace solvers {

IpoptSolver::IpoptSolver()
    : SolverBase(id(), &is_available, &is_enabled, &ProgramAttributesSatisfied),
      // The default linear solver is MA27, but it is not freely redistributable
      // so we cannot use it. MUMPS is the only compatible linear solver
      // guaranteed to be available on both macOS and Ubuntu. In versions of
      // IPOPT prior to 3.13, it would correctly determine that MUMPS was the
      // only available solver, but its behavior changed to instead error having
      // unsuccessfully tried to dlopen a nonexistent hsl library that would
      // contain MA27.
      default_linear_solver_("mumps") {}

IpoptSolver::~IpoptSolver() = default;

void IpoptSolver::SetDefaultLinearSolver(std::string linear_solver) {
  default_linear_solver_ = std::move(linear_solver);
}

SolverId IpoptSolver::id() {
  static const never_destroyed<SolverId> singleton{"IPOPT"};
  return singleton.access();
}

bool IpoptSolver::is_enabled() {
  return true;
}

bool IpoptSolver::ProgramAttributesSatisfied(const MathematicalProgram& prog) {
  static const never_destroyed<ProgramAttributes> solver_capabilities(
      std::initializer_list<ProgramAttribute>{
          ProgramAttribute::kGenericConstraint,
          ProgramAttribute::kLinearEqualityConstraint,
          ProgramAttribute::kLinearConstraint,
          ProgramAttribute::kQuadraticConstraint,
          ProgramAttribute::kLorentzConeConstraint,
          ProgramAttribute::kRotatedLorentzConeConstraint,
          ProgramAttribute::kGenericCost, ProgramAttribute::kLinearCost,
          ProgramAttribute::kL2NormCost, ProgramAttribute::kQuadraticCost,
          ProgramAttribute::kCallback});
  return AreRequiredAttributesSupported(prog.required_capabilities(),
                                        solver_capabilities.access());
}
}  // namespace solvers
}  // namespace drake
