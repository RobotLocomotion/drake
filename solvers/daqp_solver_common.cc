/* clang-format off to disable clang-format-includes */
#include "drake/solvers/daqp_solver.h"
/* clang-format on */

#include <string>

#include "drake/common/never_destroyed.h"
#include "drake/solvers/aggregate_costs_constraints.h"
#include "drake/solvers/mathematical_program.h"

// This file contains implementations that are common to both the available and
// unavailable flavor of this class.

namespace drake {
namespace solvers {

DaqpSolver::DaqpSolver()
    : SolverBase(id(), &is_available, &is_enabled, &ProgramAttributesSatisfied,
                 &UnsatisfiedProgramAttributes) {}

DaqpSolver::~DaqpSolver() = default;

SolverId DaqpSolver::id() {
  static const never_destroyed<SolverId> singleton{"DAQP"};
  return singleton.access();
}

bool DaqpSolver::is_enabled() {
  return true;
}

namespace {
// If the program is compatible with this solver, returns true and clears the
// explanation.  Otherwise, returns false and sets the explanation.  In either
// case, the explanation can be nullptr in which case it is ignored.
bool CheckAttributes(const MathematicalProgram& prog,
                     std::string* explanation) {
  static const never_destroyed<ProgramAttributes> solver_capabilities(
      std::initializer_list<ProgramAttribute>{
          ProgramAttribute::kLinearCost, ProgramAttribute::kQuadraticCost,
          ProgramAttribute::kLinearConstraint,
          ProgramAttribute::kLinearEqualityConstraint});
  if (!internal::CheckConvexSolverAttributes(prog, solver_capabilities.access(),
                                             "DaqpSolver", explanation)) {
    return false;
  }
  if (!prog.required_capabilities().contains(
          ProgramAttribute::kQuadraticCost)) {
    if (explanation) {
      *explanation =
          "DaqpSolver is unable to solve because a QuadraticCost is required"
          " but has not been declared. Please use a different solver such as"
          " CLP (for linear programming) if you don't want to add a quadratic"
          " cost to this program.";
    }
    return false;
  }
  return true;
}
}  // namespace

bool DaqpSolver::ProgramAttributesSatisfied(const MathematicalProgram& prog) {
  return CheckAttributes(prog, nullptr);
}

std::string DaqpSolver::UnsatisfiedProgramAttributes(
    const MathematicalProgram& prog) {
  std::string explanation;
  CheckAttributes(prog, &explanation);
  return explanation;
}

}  // namespace solvers
}  // namespace drake
