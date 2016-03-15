#ifndef DRAKE_NLOPT_SOLVER_H
#define DRAKE_NLOPT_SOLVER_H

#include "MathematicalProgram.h"

namespace Drake {

class MathematicalProgramNloptSolver :
      public MathematicalProgramSolverInterface  {
 public:
  // This solver is implemented in various pieces depending on if
  // NLopt was available during compilation.
  virtual bool available() const override;
  virtual bool solve(OptimizationProblem& prog) const override;
};

}

#endif
