/* clang-format off to disable clang-format-includes */
#include "drake/solvers/csdp_solver.h"
/* clang-format on */

#include "drake/common/never_destroyed.h"
#include "drake/solvers/mathematical_program.h"

// This file contains implementations that are common to both the available and
// unavailable flavor of this class.

namespace drake {
namespace solvers {

CsdpSolver::CsdpSolver(RemoveFreeVariableMethod method)
    : SolverBase(&id, &is_available, &is_enabled, &ProgramAttributesSatisfied),
      method_{method} {}

CsdpSolver::~CsdpSolver() = default;

SolverId CsdpSolver::id() {
  static const never_destroyed<SolverId> singleton{"CSDP"};
  return singleton.access();
}

bool CsdpSolver::is_enabled() { return true; }

bool CsdpSolver::ProgramAttributesSatisfied(const MathematicalProgram& prog) {
  static const never_destroyed<ProgramAttributes> solver_capabilities(
      std::initializer_list<ProgramAttribute>{
          ProgramAttribute::kLinearCost, ProgramAttribute::kLinearConstraint,
          ProgramAttribute::kLinearEqualityConstraint,
          ProgramAttribute::kLorentzConeConstraint,
          ProgramAttribute::kRotatedLorentzConeConstraint,
          ProgramAttribute::kPositiveSemidefiniteConstraint});
  return AreRequiredAttributesSupported(prog.required_capabilities(),
                                        solver_capabilities.access());
}

}  // namespace solvers
}  // namespace drake
