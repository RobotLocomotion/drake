/* clang-format off to disable clang-format-includes */
#include "drake/solvers/ibex_solver.h"
/* clang-format on */

#include "drake/common/never_destroyed.h"
#include "drake/solvers/mathematical_program.h"

// This file contains implementations that are common to both the available and
// unavailable flavor of this class.

namespace drake {
namespace solvers {

IbexSolver::IbexSolver()
    : SolverBase(&id, &is_available, &is_enabled, &ProgramAttributesSatisfied) {
}

IbexSolver::~IbexSolver() = default;

SolverId IbexSolver::id() {
  static const never_destroyed<SolverId> singleton{"IBEX"};
  return singleton.access();
}

bool IbexSolver::is_enabled() { return true; }

bool IbexSolver::ProgramAttributesSatisfied(const MathematicalProgram& prog) {
  static const never_destroyed<ProgramAttributes> solver_capabilities(
      std::initializer_list<ProgramAttribute>{
          ProgramAttribute::kExponentialConeConstraint,
          ProgramAttribute::kGenericConstraint,
          ProgramAttribute::kLinearComplementarityConstraint,
          ProgramAttribute::kLinearConstraint,
          ProgramAttribute::kLinearEqualityConstraint,
          ProgramAttribute::kLorentzConeConstraint,
          ProgramAttribute::kRotatedLorentzConeConstraint,
          ProgramAttribute::kGenericCost,
          ProgramAttribute::kLinearCost,
          ProgramAttribute::kQuadraticCost,
      });

  return AreRequiredAttributesSupported(prog.required_capabilities(),
                                        solver_capabilities.access());
}

}  // namespace solvers
}  // namespace drake
