#ifndef SNOPT_SOLVER_H
#define SNOPT_SOLVER_H

#include "drake/drakeOptimization_export.h"

#include "MathematicalProgram.h"

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
