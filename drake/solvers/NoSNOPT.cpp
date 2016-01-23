
#include "Optimization.h"

bool Drake::OptimizationProblem::NonlinearProgram::hasSNOPT() const { return false; }

bool Drake::OptimizationProblem::NonlinearProgram::solveWithSNOPT(OptimizationProblem &prog) const {
  throw std::runtime_error("The SNOPT bindings were not compiled.  You'll need to use a different solver.");
}
