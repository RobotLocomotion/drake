#pragma once

#include "drake/drakeOptimization_export.h"

#include "drake/solvers/MathematicalProgram.h"


namespace drake {
namespace solvers {

class DRAKEOPTIMIZATION_EXPORT GurobiSolver :
public MathematicalProgramSolverInterface  {
public:
// This solver is implemented in various pieces depending on if
// Gurobi was available during compilation.
bool available() const override;
drake::solvers::SolutionResult Solve(
    OptimizationProblem& prog) const override;
};

}  // end namespace solvers
}  // end namespace drake
