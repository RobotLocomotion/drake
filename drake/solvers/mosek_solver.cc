// A wrapper file for MosekWrapper and mosekQP that handles constraint and
// objective marshalling

#include "drake/solvers/mosek_solver.h"
#include "drake/solvers/mosek_wrapper.h"

#include <Eigen/Core>

namespace drake {
namespace solvers {

bool MosekSolver::available() const { return true; }

SolutionResult MosekSolver::Solve(MathematicalProgram& prog) const {
  if (!prog.GetSolverOptionsStr("Mosek").empty()) {
    if (prog.GetSolverOptionsStr("Mosek").at("problemtype").find("linear")
            != std::string::npos ||
        prog.GetSolverOptionsStr("Mosek").at("problemtype").find("quadratic")
            != std::string::npos ||
        prog.GetSolverOptionsStr("Mosek").at("problemtype").find("sdp")
            != std::string::npos) {
      return MosekWrapper::Solve(prog);
    }
  }  // TODO(alexdunyak): add more mosek solution types.
  return kUnknownError;
}

}
}
