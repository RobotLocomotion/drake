#pragma once

#include <string>

#include "drake/common/drake_copyable.h"
#include "drake/solvers/mathematical_program.h"

namespace drake {
namespace solvers {

class SnoptSolver : public MathematicalProgramSolverInterface  {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(SnoptSolver)

  SnoptSolver() = default;

  // This solver is implemented in various pieces depending on if
  // SNOPT was available during compilation.
  bool available() const override;

  std::string SolverName() const override { return "SNOPT"; }

  SolutionResult Solve(MathematicalProgram& prog) const override;
};

}  // namespace solvers
}  // namespace drake
