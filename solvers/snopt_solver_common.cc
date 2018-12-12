/* clang-format off to disable clang-format-includes */
#include "drake/solvers/snopt_solver.h"
/* clang-format on */

#include "drake/common/never_destroyed.h"
#include "drake/solvers/mathematical_program.h"

namespace drake {
namespace solvers {

SolverId SnoptSolver::solver_id() const {
  return id();
}

SolverId SnoptSolver::id() {
  static const never_destroyed<SolverId> singleton{"SNOPT"};
  return singleton.access();
}

bool SnoptSolver::AreProgramAttributesSatisfied(
    const MathematicalProgram& prog) const {
  return SnoptSolver::ProgramAttributesSatisfied(prog);
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
          ProgramAttribute::kQuadraticCost, ProgramAttribute::kCallback});
  return AreRequiredAttributesSupported(prog.required_capabilities(),
                                        solver_capabilities.access());
}

SolutionResult SnoptSolver::Solve(MathematicalProgram& prog) const {
  MathematicalProgramResult result;
  Solve(prog, {}, {}, &result);
  const SolverResult solver_result = result.ConvertToSolverResult();
  prog.SetSolverResult(solver_result);
  return result.get_solution_result();
}

}  // namespace solvers
}  // namespace drake
