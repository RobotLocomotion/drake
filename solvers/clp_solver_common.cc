/* clang-format off to disable clang-format-includes */
#include "drake/solvers/clp_solver.h"
/* clang-format on */

#include "drake/common/never_destroyed.h"
#include "drake/solvers/mathematical_program.h"

namespace drake {
namespace solvers {
ClpSolver::ClpSolver()
    : SolverBase(&id, &is_available, &is_enabled, &ProgramAttributesSatisfied) {
}

ClpSolver::~ClpSolver() = default;

SolverId ClpSolver::id() {
  static const never_destroyed<SolverId> singleton{"CLP"};
  return singleton.access();
}

bool ClpSolver::is_enabled() { return true; }

bool ClpSolver::ProgramAttributesSatisfied(const MathematicalProgram& prog) {
  static const never_destroyed<ProgramAttributes> solver_capabilities(
      std::initializer_list<ProgramAttribute>{
          ProgramAttribute::kLinearEqualityConstraint,
          ProgramAttribute::kLinearConstraint, ProgramAttribute::kLinearCost,
          ProgramAttribute::kQuadraticCost});
  return AreRequiredAttributesSupported(prog.required_capabilities(),
                                        solver_capabilities.access());
}
}  // namespace solvers
}  // namespace drake
