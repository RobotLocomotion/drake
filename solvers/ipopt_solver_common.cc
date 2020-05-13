/* clang-format off to disable clang-format-includes */
#include "drake/solvers/ipopt_solver.h"
/* clang-format on */

#include "drake/common/never_destroyed.h"
#include "drake/solvers/mathematical_program.h"

namespace drake {
namespace solvers {

IpoptSolver::IpoptSolver()
    : SolverBase(&id, &is_available, &is_enabled,
                 &ProgramAttributesSatisfied) {}

IpoptSolver::~IpoptSolver() = default;

SolverId IpoptSolver::id() {
  static const never_destroyed<SolverId> singleton{"IPOPT"};
  return singleton.access();
}

bool IpoptSolver::is_enabled() { return true; }

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
          ProgramAttribute::kQuadraticCost, ProgramAttribute::kCallback});
  return AreRequiredAttributesSupported(prog.required_capabilities(),
                                        solver_capabilities.access());
}
}  // namespace solvers
}  // namespace drake
