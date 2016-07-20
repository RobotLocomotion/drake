// Copyright 2016, Alex Dunyak
// A wrapper file for MosekLP and mosekQP that handles constraint and
// objective marshalling
#pragma once

#include <Eigen/Core>

#include "drake/solvers/Optimization.h"
#include "drake/solvers/MathematicalProgram.h"
#include "drake/solvers/solution_result.h"

namespace drake {
namespace solvers {

/** MosekSolver
* A wrapper class that calls the correct version of MosekLP or (eventually)
* MosekQP. The functions are defined in the relevant .h files if mosek is
* included.
*/
class DRAKEOPTIMIZATION_EXPORT MosekSolver :
    public MathematicalProgramSolverInterface {
 public:
  /** available()
  * Defined true if Mosek was included during compilation, false otherwise.
  */
  bool available() const override;

  SolutionResult Solve(OptimizationProblem& prog) const override;
};

}
}
