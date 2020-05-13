/* clang-format off to disable clang-format-includes */
#include "drake/solvers/mosek_solver.h"
/* clang-format on */

#include <cstdlib>
#include <cstring>

#include "drake/common/never_destroyed.h"
#include "drake/solvers/mathematical_program.h"

namespace drake {
namespace solvers {

MosekSolver::MosekSolver()
    : SolverBase(&id, &is_available, &is_enabled,
                 &ProgramAttributesSatisfied) {}

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

bool MosekSolver::ProgramAttributesSatisfied(const MathematicalProgram& prog) {
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
  return AreRequiredAttributesSupported(prog.required_capabilities(),
                                        solver_capabilities.access());
}

}  // namespace solvers
}  // namespace drake
