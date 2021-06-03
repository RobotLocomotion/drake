/* clang-format off to disable clang-format-includes */
#include "drake/solvers/osqp_solver.h"
/* clang-format on */

#include "drake/common/never_destroyed.h"
#include "drake/solvers/aggregate_costs_constraints.h"
#include "drake/solvers/mathematical_program.h"

// This file contains implementations that are common to both the available and
// unavailable flavor of this class.

namespace drake {
namespace solvers {

OsqpSolver::OsqpSolver()
    : SolverBase(&id, &is_available, &is_enabled, &ProgramAttributesSatisfied,
                 &UnsatisfiedProgramAttributes) {}

OsqpSolver::~OsqpSolver() = default;

SolverId OsqpSolver::id() {
  static const never_destroyed<SolverId> singleton{"OSQP"};
  return singleton.access();
}

bool OsqpSolver::is_enabled() { return true; }

namespace {
// If the program is compatible with this solver, returns true and clears the
// explanation.  Otherwise, returns false and sets the explanation.  In either
// case, the explanation can be nullptr in which case it is ignored.
bool CheckAttributes(
    const MathematicalProgram& prog,
    std::string* explanation) {
  static const never_destroyed<ProgramAttributes> solver_capabilities(
      std::initializer_list<ProgramAttribute>{
          ProgramAttribute::kLinearCost,
          ProgramAttribute::kQuadraticCost,
          ProgramAttribute::kLinearConstraint,
          ProgramAttribute::kLinearEqualityConstraint});
  const ProgramAttributes& required_capabilities = prog.required_capabilities();
  const bool capabilities_match = AreRequiredAttributesSupported(
      required_capabilities, solver_capabilities.access(), explanation);
  if (!capabilities_match) {
    if (explanation) {
      *explanation = fmt::format(
          "OsqpSolver is unable to solve because {}.", *explanation);
    }
    return false;
  }
  if (required_capabilities.count(ProgramAttribute::kQuadraticCost) == 0) {
    if (explanation) {
      *explanation =
          "OsqpSolver is unable to solve because a QuadraticCost is required"
          " but has not beed declared; OSQP works best with a quadratic cost."
          " Please use a different solver such as CLP (for linear programming)"
          " or IPOPT/SNOPT (for nonlinear programming) if you don't want to add"
          " a quadratic cost to this program.";
    }
    return false;
  }
  const Binding<QuadraticCost>* nonconvex_quadratic_cost =
      FindNonconvexQuadraticCost(prog.quadratic_costs());
  if (nonconvex_quadratic_cost != nullptr) {
    if (explanation) {
      *explanation =
          "OsqpSolver is unable to solve because the quadratic cost " +
          nonconvex_quadratic_cost->to_string() +
          " is non-convex. Either change this cost to a convex one, or switch "
          "to a different solver like SNOPT/IPOPT/NLOPT.";
    }
    return false;
  }
  if (explanation) {
    explanation->clear();
  }
  return true;
}
}  // namespace

bool OsqpSolver::ProgramAttributesSatisfied(const MathematicalProgram& prog) {
  return CheckAttributes(prog, nullptr);
}

std::string OsqpSolver::UnsatisfiedProgramAttributes(
    const MathematicalProgram& prog) {
  std::string explanation;
  CheckAttributes(prog, &explanation);
  return explanation;
}

}  // namespace solvers
}  // namespace drake
