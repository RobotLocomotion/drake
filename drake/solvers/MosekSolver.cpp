// A wrapper file for MosekWrapper and mosekQP that handles constraint and
// objective marshalling

#include "drake/solvers/MosekSolver.h"
#include "drake/solvers/MosekWrapper.h"

#include <Eigen/Core>

#include "drake/solvers/Optimization.h"
#include "drake/solvers/MathematicalProgram.h"
#include "drake/solvers/solution_result.h"

namespace drake {
namespace solvers {

bool MosekSolver::available() const { return true; }

SolutionResult MosekSolver::Solve(OptimizationProblem& prog) const {
  if (!prog.GetSolverOptionsStr("Mosek").empty()) {
    if (prog.GetSolverOptionsStr("Mosek").at("problemtype").find("linear")
            != std::string::npos ||
        prog.GetSolverOptionsStr("Mosek").at("problemtype").find("quadratic")
            != std::string::npos) {
      return MosekWrapper::Solve(prog);
    }
  }  // TODO(alexdunyak): add more mosek solution types.
  return kUnknownError;
}

}
}
