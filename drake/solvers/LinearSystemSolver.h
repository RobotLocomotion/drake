#pragma once

#include "drake/drakeOptimization_export.h"

#include "drake/solvers/MathematicalProgram.h"

namespace drake {
namespace solvers {

class DRAKEOPTIMIZATION_EXPORT LinearSystemSolver :
      public MathematicalProgramSolverInterface  {
 public:
  bool available() const override;
  SolutionResult Solve(OptimizationProblem& prog) const override;
};

}  // namespace solvers
}  // namespace drake
