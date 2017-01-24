#pragma once

#include <string>

#include "drake/common/drake_copyable.h"
#include "drake/solvers/mathematical_program.h"

namespace drake {
namespace solvers {

class LinearSystemSolver : public MathematicalProgramSolverInterface {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(LinearSystemSolver)

  LinearSystemSolver() = default;

  bool available() const override;

  std::string SolverName() const override { return "Linear System Solver"; }

  SolutionResult Solve(MathematicalProgram& prog) const override;
};

}  // namespace solvers
}  // namespace drake
