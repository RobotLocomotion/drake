/* clang-format off to disable clang-format-includes */
#include "drake/solvers/scs_solver.h"
/* clang-format on */

#include "drake/common/never_destroyed.h"
#include "drake/solvers/mathematical_program.h"

namespace drake {
namespace solvers {

SolverId ScsSolver::solver_id() const {
  return id();
}

SolverId ScsSolver::id() {
  static const never_destroyed<SolverId> singleton{"SCS"};
  return singleton.access();
}

bool ScsSolver::AreProgramAttributesSatisfied(
    const MathematicalProgram& prog) const {
  return ScsSolver::ProgramAttributesSatisfied(prog);
}

bool ScsSolver::ProgramAttributesSatisfied(const MathematicalProgram& prog) {
  return AreRequiredAttributesSupported(
      prog.required_capabilities(),
      ProgramAttributes({ProgramAttribute::kLinearEqualityConstraint,
                         ProgramAttribute::kLinearConstraint,
                         ProgramAttribute::kLorentzConeConstraint,
                         ProgramAttribute::kRotatedLorentzConeConstraint,
                         ProgramAttribute::kPositiveSemidefiniteConstraint,
                         ProgramAttribute::kLinearCost,
                         ProgramAttribute::kQuadraticCost}));
}

}  // namespace solvers
}  // namespace drake
