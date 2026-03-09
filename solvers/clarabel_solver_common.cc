/* clang-format off to disable clang-format-includes */
#include "drake/solvers/clarabel_solver.h"
/* clang-format on */

#include <string>

#include "drake/common/never_destroyed.h"
#include "drake/solvers/aggregate_costs_constraints.h"
#include "drake/solvers/mathematical_program.h"

namespace drake {
namespace solvers {
ClarabelSolver::ClarabelSolver()
    : SolverBase(id(), &is_available, &is_enabled, &ProgramAttributesSatisfied,
                 &UnsatisfiedProgramAttributes) {}

ClarabelSolver::~ClarabelSolver() = default;

SolverId ClarabelSolver::id() {
  static const never_destroyed<SolverId> singleton{"Clarabel"};
  return singleton.access();
}

bool ClarabelSolver::is_enabled() {
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
          ProgramAttribute::kLinearEqualityConstraint,
          ProgramAttribute::kLinearConstraint,
          ProgramAttribute::kLorentzConeConstraint,
          ProgramAttribute::kRotatedLorentzConeConstraint,
          ProgramAttribute::kPositiveSemidefiniteConstraint,
          ProgramAttribute::kExponentialConeConstraint,
          ProgramAttribute::kLinearCost, ProgramAttribute::kQuadraticCost,
          ProgramAttribute::kL2NormCost});
  return internal::CheckConvexSolverAttributes(
      prog, solver_capabilities.access(), "ClarabelSolver", explanation);
}
}  // namespace

bool ClarabelSolver::ProgramAttributesSatisfied(
    const MathematicalProgram& prog) {
  return CheckAttributes(prog, nullptr);
}

std::string ClarabelSolver::UnsatisfiedProgramAttributes(
    const MathematicalProgram& prog) {
  std::string explanation;
  CheckAttributes(prog, &explanation);
  return explanation;
}

}  // namespace solvers
}  // namespace drake
