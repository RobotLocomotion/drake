#pragma once

#include "drake/drakeOptimization_export.h"

#include "drake/solvers/MathematicalProgram.h"

namespace drake {
namespace solvers {

class DRAKEOPTIMIZATION_EXPORT IpoptSolver :
      public Drake::MathematicalProgramSolverInterface  {
 public:
  // This solver is implemented in various pieces depending on if
  // Ipopt was available during compilation.
  bool available() const override;
  drake::solvers::SolutionResult Solve(
      Drake::OptimizationProblem& prog) const override;
};

}  // namespace drake
}  // namespace solvers
