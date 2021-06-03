/* clang-format off to disable clang-format-includes */
#include "drake/solvers/clp_solver.h"
/* clang-format on */

#include "drake/common/never_destroyed.h"
#include "drake/solvers/aggregate_costs_constraints.h"
#include "drake/solvers/mathematical_program.h"

namespace drake {
namespace solvers {
ClpSolver::ClpSolver()
    : SolverBase(&id, &is_available, &is_enabled, &ProgramAttributesSatisfied,
                 &UnsatisfiedProgramAttributes) {}

ClpSolver::~ClpSolver() = default;

SolverId ClpSolver::id() {
  static const never_destroyed<SolverId> singleton{"CLP"};
  return singleton.access();
}

bool ClpSolver::is_enabled() { return true; }

namespace {
// If the program is compatible with this solver, returns true and clears the
// explanation.  Otherwise, returns false and sets the explanation.  In either
// case, the explanation can be nullptr in which case it is ignored.
bool CheckAttributes(const MathematicalProgram& prog,
                     std::string* explanation) {
  static const never_destroyed<ProgramAttributes> solver_capabilities(
      std::initializer_list<ProgramAttribute>{
          ProgramAttribute::kLinearEqualityConstraint,
          ProgramAttribute::kLinearConstraint, ProgramAttribute::kLinearCost,
          ProgramAttribute::kQuadraticCost});
  return internal::CheckConvexSolverAttributes(
      prog, solver_capabilities.access(), "ClpSolver", explanation);
}
}  // namespace

bool ClpSolver::ProgramAttributesSatisfied(const MathematicalProgram& prog) {
  return CheckAttributes(prog, nullptr);
}

std::string ClpSolver::UnsatisfiedProgramAttributes(
    const MathematicalProgram& prog) {
  std::string explanation;
  CheckAttributes(prog, &explanation);
  return explanation;
}
}  // namespace solvers
}  // namespace drake
