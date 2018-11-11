/* clang-format off to disable clang-format-includes */
#include "drake/solvers/dreal_solver.h"
/* clang-format on */

#include "drake/common/never_destroyed.h"
#include "drake/solvers/mathematical_program.h"

// This file contains implementations that are common to both the available and
// unavailable flavor of this class.

namespace drake {
namespace solvers {

SolverId DrealSolver::solver_id() const {
  return id();
}

SolverId DrealSolver::id() {
  static const never_destroyed<SolverId> singleton{"dReal"};
  return singleton.access();
}

bool DrealSolver::AreProgramAttributesSatisfied(
    const MathematicalProgram& prog) const {
  return DrealSolver::ProgramAttributesSatisfied(prog);
}

bool DrealSolver::ProgramAttributesSatisfied(const MathematicalProgram& prog) {
  static const never_destroyed<ProgramAttributes> solver_capabilities(
      std::initializer_list<ProgramAttribute>{
          ProgramAttribute::kGenericConstraint,
          ProgramAttribute::kLinearEqualityConstraint,
          ProgramAttribute::kLinearConstraint,
          ProgramAttribute::kLorentzConeConstraint,
          ProgramAttribute::kRotatedLorentzConeConstraint,
          ProgramAttribute::kLinearComplementarityConstraint,
          ProgramAttribute::kLinearCost, ProgramAttribute::kQuadraticCost,
          ProgramAttribute::kBinaryVariable});
  return AreRequiredAttributesSupported(prog.required_capabilities(),
                                        solver_capabilities.access());
}

}  // namespace solvers
}  // namespace drake
