/* clang-format off to disable clang-format-includes */
#include "drake/solvers/scs_solver.h"
/* clang-format on */

#include "drake/common/never_destroyed.h"
#include "drake/solvers/aggregate_costs_constraints.h"
#include "drake/solvers/mathematical_program.h"

namespace drake {
namespace solvers {

ScsSolver::ScsSolver()
    : SolverBase(&id, &is_available, &is_enabled,
                 &ProgramAttributesSatisfied) {}

ScsSolver::~ScsSolver() = default;

SolverId ScsSolver::id() {
  static const never_destroyed<SolverId> singleton{"SCS"};
  return singleton.access();
}

bool ScsSolver::is_enabled() { return true; }

bool ScsSolver::ProgramAttributesSatisfied(const MathematicalProgram& prog) {
  static const never_destroyed<ProgramAttributes> solver_capabilities(
      std::initializer_list<ProgramAttribute>{
          ProgramAttribute::kLinearEqualityConstraint,
          ProgramAttribute::kLinearConstraint,
          ProgramAttribute::kLorentzConeConstraint,
          ProgramAttribute::kRotatedLorentzConeConstraint,
          ProgramAttribute::kPositiveSemidefiniteConstraint,
          ProgramAttribute::kExponentialConeConstraint,
          ProgramAttribute::kLinearCost, ProgramAttribute::kQuadraticCost});
  return AreRequiredAttributesSupported(prog.required_capabilities(),
                                        solver_capabilities.access()) &&
         AreAllQuadraticCostsConvex(prog.quadratic_costs());
}

}  // namespace solvers
}  // namespace drake
