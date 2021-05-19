/* clang-format off to disable clang-format-includes */
#include "drake/solvers/mosek_solver.h"
/* clang-format on */

#include <cstdlib>
#include <cstring>

#include "drake/common/never_destroyed.h"
#include "drake/solvers/aggregate_costs_constraints.h"
#include "drake/solvers/mathematical_program.h"

namespace drake {
namespace solvers {

MosekSolver::MosekSolver()
    : SolverBase(&id, &is_available, &is_enabled, &ProgramAttributesSatisfied,
                 &UnsatisfiedProgramAttributes) {}

MosekSolver::~MosekSolver() = default;

SolverId MosekSolver::id() {
  static const never_destroyed<SolverId> singleton{"Mosek"};
  return singleton.access();
}

bool MosekSolver::is_enabled() {
  const char* moseklm_license_file = std::getenv("MOSEKLM_LICENSE_FILE");
  return ((moseklm_license_file != nullptr) &&
          (std::strlen(moseklm_license_file) > 0));
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
          ProgramAttribute::kBinaryVariable});
  return internal::CheckConvexSolverAttributes(
      prog, solver_capabilities.access(), "MosekSolver", explanation);
}
}  // namespace

bool MosekSolver::ProgramAttributesSatisfied(const MathematicalProgram& prog) {
  return CheckAttributes(prog, nullptr);
}

std::string MosekSolver::UnsatisfiedProgramAttributes(
    const MathematicalProgram& prog) {
  std::string explanation;
  CheckAttributes(prog, &explanation);
  return explanation;
}

}  // namespace solvers
}  // namespace drake
