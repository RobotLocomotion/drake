/* clang-format off to disable clang-format-includes */
#include "drake/solvers/scs_solver.h"
/* clang-format on */

#include "drake/common/never_destroyed.h"
#include "drake/solvers/aggregate_costs_constraints.h"
#include "drake/solvers/mathematical_program.h"

namespace drake {
namespace solvers {

ScsSolver::ScsSolver()
    : SolverBase(&id, &is_available, &is_enabled, &ProgramAttributesSatisfied,
                 &UnsatisfiedProgramAttributes) {}

ScsSolver::~ScsSolver() = default;

SolverId ScsSolver::id() {
  static const never_destroyed<SolverId> singleton{"SCS"};
  return singleton.access();
}

bool ScsSolver::is_enabled() { return true; }

namespace {
// If the program is compatible with this solver, returns true and clears the
// explanation.  Otherwise, returns false and sets the explanation.  In either
// case, the explanation can be nullptr in which case it is ignored.
bool CheckAttributes(const MathematicalProgram& prog,
                     std::string* explanation) {
  static const never_destroyed<ProgramAttributes> solver_capabilities(
      std::initializer_list<ProgramAttribute>{
          ProgramAttribute::kLinearEqualityConstraint,
          ProgramAttribute::kLinearConstraint,
          ProgramAttribute::kLorentzConeConstraint,
          ProgramAttribute::kRotatedLorentzConeConstraint,
          ProgramAttribute::kPositiveSemidefiniteConstraint,
          ProgramAttribute::kExponentialConeConstraint,
          ProgramAttribute::kLinearCost, ProgramAttribute::kQuadraticCost});
  return internal::CheckConvexSolverAttributes(
      prog, solver_capabilities.access(), "ScsSolver", explanation);
}
}  // namespace

bool ScsSolver::ProgramAttributesSatisfied(const MathematicalProgram& prog) {
  return CheckAttributes(prog, nullptr);
}

std::string ScsSolver::UnsatisfiedProgramAttributes(
    const MathematicalProgram& prog) {
  std::string explanation;
  CheckAttributes(prog, &explanation);
  return explanation;
}

}  // namespace solvers
}  // namespace drake
