#pragma once

#include "drake/solvers/mathematical_program.h"

namespace drake {
namespace solvers {

class EqualityConstrainedQPSolver : public MathematicalProgramSolverInterface {
 public:
  bool available() const override;
  SolutionResult Solve(MathematicalProgram& prog) const override;
};

}  // namespace solvers
}  // namespace drake
