/* clang-format off to disable clang-format-includes */
#include "drake/solvers/nlopt_solver.h"
/* clang-format on */

#include "drake/common/never_destroyed.h"
#include "drake/solvers/mathematical_program.h"

namespace drake {
namespace solvers {

NloptSolver::NloptSolver()
    : SolverBase(&id, &is_available, &is_enabled,
                 &ProgramAttributesSatisfied) {}

NloptSolver::~NloptSolver() = default;

SolverId NloptSolver::id() {
  static const never_destroyed<SolverId> singleton{"NLopt"};
  return singleton.access();
}

bool NloptSolver::is_enabled() { return true; }

bool NloptSolver::ProgramAttributesSatisfied(const MathematicalProgram& prog) {
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

std::string NloptSolver::ConstraintToleranceName() { return "constraint_tol"; }

std::string NloptSolver::XRelativeToleranceName() { return "xtol_rel"; }

std::string NloptSolver::XAbsoluteToleranceName() { return "xtol_abs"; }

std::string NloptSolver::MaxEvalName() { return "max_eval"; }

}  // namespace solvers
}  // namespace drake
