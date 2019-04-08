/* clang-format off to disable clang-format-includes */
#include "drake/solvers/scs_solver.h"
/* clang-format on */

#include "drake/common/never_destroyed.h"
#include "drake/solvers/mathematical_program.h"

namespace drake {
namespace solvers {

ScsSolver::ScsSolver()
    : SolverBase(&id, &is_available, &ProgramAttributesSatisfied) {}

ScsSolver::~ScsSolver() = default;

SolverId ScsSolver::id() {
  static const never_destroyed<SolverId> singleton{"SCS"};
  return singleton.access();
}

bool ScsSolver::ProgramAttributesSatisfied(const MathematicalProgram& prog) {
  return AreRequiredAttributesSupported(
      prog.required_capabilities(),
      ProgramAttributes({ProgramAttribute::kLinearEqualityConstraint,
                         ProgramAttribute::kLinearConstraint,
                         ProgramAttribute::kLorentzConeConstraint,
                         ProgramAttribute::kRotatedLorentzConeConstraint,
                         ProgramAttribute::kPositiveSemidefiniteConstraint,
                         ProgramAttribute::kExponentialConeConstraint,
                         ProgramAttribute::kLinearCost,
                         ProgramAttribute::kQuadraticCost}));
}

}  // namespace solvers
}  // namespace drake
