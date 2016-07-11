// Copyright 2016, Alex Dunyak
// A wrapper file for MosekLP and mosekQP that handles constraint and
// objective marshalling


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
  bool available() const override;
  SolutionResult LPSolve(OptimizationProblem& prog) const;
  SolutionResult Solve(OptimizationProblem& prog) const override {
    if (!prog.GetSolverOptionsStr("Mosek").empty()) {
      if (prog.GetSolverOptionsStr("Mosek").at("problemtype").find("linear")
          != std::string::npos) {
        return LPSolve(prog);
      }
    }  // TODO(alexdunyak): add more mosek solution types.
    return kUnknownError;
  }
};

}
}
