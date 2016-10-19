// A wrapper file for MosekWrapper and mosekQP that handles constraint and
// objective marshalling

#include "drake/solvers/mosek_solver.h"

#include <mosek/mosek.h>

#include <Eigen/Core>

namespace drake {
namespace solvers {

// Add LinearConstraints and LinearEqualityConstraints to the Mosek task.
MSKrescodee AddLinearConstraints(MSKenv_t &env, MSKtask_t &task, const MathematicalProgram &prog) {
  for(const auto &binding : prog.linear_equality_constraints()) {

  }
}

bool MosekSolver::available() const { return true; }

SolutionResult MosekSolver::Solve(MathematicalProgram& prog) const {
  MSKenv_t env = NULL;
  MSKtask_t task = NULL;
  MSKrescodee rescode;

  // Create the Mosek environment.
  rescode = MSK_makeenv(&env, NULL);
  if (rescode == MSK_RES_OK) {
    // Create the optimization task.
    rescode = MSK_maketask(env, 0, prog.num_vars(), &task);
  }
  // Add linear constraints
  rescode = AddLinearConstraints(env, task, prog);
}

}
}
