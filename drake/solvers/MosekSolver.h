// Copyright 2016, Alex Dunyak
// A wrapper file for mosekLP and mosekQP that handles constraint and
// objective marshalling

#include "drake/solvers/mosekLP.h"

#include <Eigen/Core>

#include "drake/solvers/Optimization.h"
#include "drake/solvers/MathematicalProgram.h"
#include "drake/solvers/solution_result.h"

namespace drake {
namespace solvers {
class DRAKEOPTIMIZATION_EXPORT MosekSolver :
    public MathematicalProgramSolverInterface {
 public:
  bool available() const override { return true; }
  SolutionResult Solve(OptimizationProblem& prog) const override {
    if (!prog.GetSolverOptionsStr("Mosek").empty()) {
      if (prog.GetSolverOptionsStr("Mosek").at("problemtype").find("linear")
          != std::string::npos) {
        mosekLP msk;
        return msk.Solve(prog);
      }
    } else {
      return kUnknownError;
    }  // TODO(alexdunyak): add more mosek solution types.
    return kUnknownError;
  }
};

}
}
