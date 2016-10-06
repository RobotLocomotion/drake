#pragma once

#include "drake/common/drake_export.h"
#include "drake/solvers/mathematical_program.h"

namespace drake {
namespace solvers {

class DRAKE_EXPORT DrealSolver
    : public MathematicalProgramSolverInterface {
 public:
  // This solver is implemented in various pieces depending on if
  // Dreal was available during compilation.
  bool available() const override;
  SolutionResult Solve(MathematicalProgram& prog) const override;
};

}  // namespace solvers
}  // namespace drake
