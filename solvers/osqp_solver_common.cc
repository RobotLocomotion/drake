/* clang-format off to disable clang-format-includes */
#include "drake/solvers/osqp_solver.h"
/* clang-format on */

#include "drake/common/never_destroyed.h"
#include "drake/common/text_logging.h"
#include "drake/solvers/mathematical_program.h"

// This file contains implementations that are common to both the available and
// unavailable flavor of this class.

namespace drake {
namespace solvers {

OsqpSolver::OsqpSolver()
    : SolverBase(&id, &is_available, &is_enabled,
                 &ProgramAttributesSatisfied) {}

OsqpSolver::~OsqpSolver() = default;

SolverId OsqpSolver::id() {
  static const never_destroyed<SolverId> singleton{"OSQP"};
  return singleton.access();
}

bool OsqpSolver::is_enabled() { return true; }

bool OsqpSolver::ProgramAttributesSatisfied(const MathematicalProgram& prog) {
  static const never_destroyed<ProgramAttributes> solver_capabilities(
      std::initializer_list<ProgramAttribute>{
          ProgramAttribute::kLinearCost, ProgramAttribute::kQuadraticCost,
          ProgramAttribute::kLinearConstraint,
          ProgramAttribute::kLinearEqualityConstraint});
  return AreRequiredAttributesSupported(prog.required_capabilities(),
                                        solver_capabilities.access()) &&
         prog.required_capabilities().count(ProgramAttribute::kQuadraticCost) >
             0;
}

void OsqpSolver::SuggestIfCapaibilitiesNotMeet(
    const MathematicalProgram& prog) const {
  if (prog.required_capabilities().count(ProgramAttribute::kQuadraticCost) ==
      0) {
    drake::log()->info(
        "Your program doesn't have a quadratic cost. OSQP works better with a "
        "quadratic cost. Please switch to other solvers like CLP (for linear "
        "programming) or IPOPT/SNOPT (for nonlinear programming) if you don't "
        "want to add a quadratic cost to this program");
  }
}

}  // namespace solvers
}  // namespace drake
