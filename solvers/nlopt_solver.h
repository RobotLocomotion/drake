#pragma once

#include <string>

#include "drake/common/drake_copyable.h"
#include "drake/solvers/mathematical_program_solver_interface.h"

namespace drake {
namespace solvers {

class NloptSolver : public MathematicalProgramSolverInterface {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(NloptSolver)

  NloptSolver() = default;
  ~NloptSolver() override = default;

  // This solver is implemented in various pieces depending on if
  // NLOpt was available during compilation.
  bool available() const override { return is_available(); };

  static bool is_available();

  SolutionResult Solve(MathematicalProgram& prog) const override;

  SolverId solver_id() const override;

  /// @return same as MathematicalProgramSolverInterface::solver_id()
  static SolverId id();

  bool AreProgramAttributesSatisfied(
      const MathematicalProgram& prog) const override;

  static bool ProgramAttributesSatisfied(const MathematicalProgram& prog);
};

}  // namespace solvers
}  // namespace drake
