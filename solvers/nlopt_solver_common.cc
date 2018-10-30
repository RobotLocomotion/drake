/* clang-format off to disable clang-format-includes */
#include "drake/solvers/nlopt_solver.h"
/* clang-format on */

#include "drake/common/never_destroyed.h"
#include "drake/solvers/mathematical_program.h"

namespace drake {
namespace solvers {

SolverId NloptSolver::solver_id() const {
  return id();
}

SolverId NloptSolver::id() {
  static const never_destroyed<SolverId> singleton{"NLopt"};
  return singleton.access();
}

bool NloptSolver::IsProgramAttributesSatisfied(
    const MathematicalProgram& prog) const {
  return NloptSolver::ProgramAttributesSatisfied(prog);
}

bool NloptSolver::ProgramAttributesSatisfied(const MathematicalProgram& prog) {
  static const never_destroyed<ProgramAttributes> solver_capabilities(
      std::initializer_list<ProgramAttribute>{
          ProgramAttribute::kGenericConstraint,
          ProgramAttribute::kLinearEqualityConstraint,
          ProgramAttribute::kLinearConstraint,
          ProgramAttribute::kLorentzConeConstraint,
          ProgramAttribute::kRotatedLorentzConeConstraint,
          ProgramAttribute::kPositiveSemidefiniteConstraint,
          ProgramAttribute::kGenericCost, ProgramAttribute::kLinearCost,
          ProgramAttribute::kQuadraticCost,
      });
  return IsSubsetOfAnotherProgramAttributes(prog.required_capabilities(),
                                            solver_capabilities.access());
}
}  // namespace solvers
}  // namespace drake
