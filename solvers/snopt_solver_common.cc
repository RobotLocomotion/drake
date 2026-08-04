/* clang-format off to disable clang-format-includes */
#include "drake/solvers/snopt_solver.h"
/* clang-format on */

#include <string>

#include "drake/common/never_destroyed.h"
#include "drake/solvers/mathematical_program.h"

namespace drake {
namespace solvers {

SnoptSolver::SnoptSolver()
    : SolverBase(id(), &is_available, &is_enabled,
                 &ProgramAttributesSatisfied) {}

SnoptSolver::~SnoptSolver() = default;

SolverId SnoptSolver::id() {
  static const never_destroyed<SolverId> singleton{"SNOPT"};
  return singleton.access();
}

bool SnoptSolver::is_enabled() {
  const char* snopt_solver_enabled = std::getenv("DRAKE_SNOPT_SOLVER_ENABLED");
  if (snopt_solver_enabled == nullptr) {
    return true;
  }
  return (std::string(snopt_solver_enabled) != "0");
}

bool SnoptSolver::ProgramAttributesSatisfied(const MathematicalProgram& prog) {
  static const never_destroyed<ProgramAttributes> solver_capabilities(
      std::initializer_list<ProgramAttribute>{
          ProgramAttribute::kGenericConstraint,
          ProgramAttribute::kLinearEqualityConstraint,
          ProgramAttribute::kLinearConstraint,
          ProgramAttribute::kQuadraticConstraint,
          ProgramAttribute::kLorentzConeConstraint,
          ProgramAttribute::kRotatedLorentzConeConstraint,
          ProgramAttribute::kLinearComplementarityConstraint,
          ProgramAttribute::kGenericCost, ProgramAttribute::kLinearCost,
          ProgramAttribute::kL2NormCost, ProgramAttribute::kQuadraticCost,
          ProgramAttribute::kCallback});
  return AreRequiredAttributesSupported(prog.required_capabilities(),
                                        solver_capabilities.access());
}

}  // namespace solvers
}  // namespace drake
