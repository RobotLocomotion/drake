#ifndef DRAKE_NLOPT_SOLVER_H
#define DRAKE_NLOPT_SOLVER_H

#include "drake/drakeOptimization_export.h"

#include "MathematicalProgram.h"

namespace Drake {

class DRAKEOPTIMIZATION_EXPORT NloptSolver :
      public MathematicalProgramSolverInterface  {
 public:
  // This solver is implemented in various pieces depending on if
  // NLOpt was available during compilation.
  virtual bool available() const override;
  virtual bool solve(OptimizationProblem& prog) const override;
};
}

#endif
