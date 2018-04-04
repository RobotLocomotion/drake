#pragma once

#include <string>

#include "drake/common/drake_copyable.h"
#include "drake/solvers/mathematical_program_solver_interface.h"

namespace drake {
namespace solvers {

class SnoptSolver : public MathematicalProgramSolverInterface  {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(SnoptSolver)

  SnoptSolver() = default;
  ~SnoptSolver() override = default;

  // This solver is implemented in various pieces depending on if
  // SNOPT was available during compilation.
  bool available() const override;

  SolutionResult Solve(MathematicalProgram& prog) const override;

  SolverId solver_id() const override;

  /// @return same as MathematicalProgramSolverInterface::solver_id()
  static SolverId id();
};

}  // namespace solvers
}  // namespace drake
