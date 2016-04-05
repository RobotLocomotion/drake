#ifndef DRAKE_SOLVERS_SNOPT_SOLVER_H_
#define DRAKE_SOLVERS_SNOPT_SOLVER_H_

#include "drake/drakeOptimization_export.h"

#include "drake/solvers/MathematicalProgram.h"

namespace Drake {

class DRAKEOPTIMIZATION_EXPORT SnoptSolver :
    public MathematicalProgramSolverInterface  {
  public:
    // This solver is implemented in various pieces depending on if
    // SNOPT was available during compilation.
    virtual bool available() const override;
    virtual bool Solve(OptimizationProblem& prog) const override;
};
}

#endif
