/* clang-format off to disable clang-format-includes */
#include "drake/solvers/osqp_solver.h"
/* clang-format on */

#include "drake/common/never_destroyed.h"
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

bool OsqpSolver::ProgramAttributesSatisfied(
    const MathematicalProgram& prog, std::string* error_message) {
  static const never_destroyed<ProgramAttributes> solver_capabilities(
      std::initializer_list<ProgramAttribute>{
          ProgramAttribute::kLinearCost,
          ProgramAttribute::kQuadraticCost,
          ProgramAttribute::kLinearConstraint,
          ProgramAttribute::kLinearEqualityConstraint});

  const ProgramAttributes& required_capabilities = prog.required_capabilities();
  const bool capabilities_match = AreRequiredAttributesSupported(
      required_capabilities, solver_capabilities.access(), error_message);
  if (!capabilities_match) {
    return false;
  }
  if (required_capabilities.count(ProgramAttribute::kQuadraticCost) == 0) {
    if (error_message) {
      *error_message =
          "a QuadraticCost is required but has not beed declared;"
          " OSQP works best with a quadratic cost."
          " Please use a different solver such as CLP (for linear programming)"
          " or IPOPT/SNOPT (for nonlinear programming) if you don't want to add"
          " a quadratic cost to this program.";
    }
    return false;
  }
  if (error_message) {
    error_message->clear();
  }
  return true;
}

}  // namespace solvers
}  // namespace drake
