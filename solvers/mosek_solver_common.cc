/* clang-format off to disable clang-format-includes */
#include "drake/solvers/mosek_solver.h"
/* clang-format on */

#include <cstdlib>
#include <cstring>

#include "drake/common/never_destroyed.h"
#include "drake/common/text_logging.h"
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
  const bool supported = AreRequiredAttributesSupported(
      prog.required_capabilities(), solver_capabilities.access());
  if (!supported) {
    return false;
  }
  // When the program has quadratic cost and nonlinear constraints, the user
  // should convert the quadratic cost to second order cone constraint using a
  // slack variable.
  if (prog.required_capabilities().count(ProgramAttribute::kQuadraticCost) >
          0 &&
      (prog.required_capabilities().count(
           ProgramAttribute::kLorentzConeConstraint) > 0 ||
       prog.required_capabilities().count(
           ProgramAttribute::kRotatedLorentzConeConstraint) > 0 ||
       prog.required_capabilities().count(
           ProgramAttribute::kPositiveSemidefiniteConstraint) > 0 ||
       prog.required_capabilities().count(
           ProgramAttribute::kExponentialConeConstraint) > 0)) {
    drake::log()->warn(
        "The program contains quadratic cost and nonlinear convex conic "
        "constraints. Consider to re-formulate the quadratic cost with a "
        "slack variable and Lorentz cone constraint. Refer to "
        "https://docs.mosek.com/modeling-cookbook/qcqo.html for more "
        "details");
    return false;
  }
  return true;
}

}  // namespace solvers
}  // namespace drake
