
#include <stdexcept>
#include "MathematicalProgram.h"

bool Drake::MathematicalProgramSNOPTSolver::available() const {
  return false;
}

bool Drake::MathematicalProgramSNOPTSolver::solve(
    OptimizationProblem &prog) const {
  throw std::runtime_error(
      "The SNOPT bindings were not compiled.  You'll need to use a different "
      "solver.");
}
