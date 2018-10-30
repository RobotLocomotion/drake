/* clang-format off to disable clang-format-includes */
#include "drake/solvers/ipopt_solver.h"
/* clang-format on */

#include "drake/common/never_destroyed.h"
#include "drake/solvers/mathematical_program.h"

namespace drake {
namespace solvers {

SolverId IpoptSolver::solver_id() const {
  return id();
}

SolverId IpoptSolver::id() {
  static const never_destroyed<SolverId> singleton{"IPOPT"};
  return singleton.access();
}

bool IpoptSolver::IsProgramAttributesSatisfied(
    const MathematicalProgram& prog) const {
  return IpoptSolver::ProgramAttributesSatisfied(prog);
}

bool IpoptSolver::ProgramAttributesSatisfied(const MathematicalProgram& prog) {
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
