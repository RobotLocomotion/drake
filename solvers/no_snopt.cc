/* clang-format off to disable clang-format-includes */
#include "drake/solvers/snopt_solver.h"
/* clang-format on */

#include <stdexcept>

namespace drake {
namespace solvers {

bool SnoptSolver::available() const { return false; }

SolutionResult SnoptSolver::Solve(MathematicalProgram&) const {
  throw std::runtime_error(
      "The SNOPT bindings were not compiled.  You'll need to use a different "
      "solver.");
}

void SnoptSolver::DoSolve(const MathematicalProgram&,
                          const Eigen::Ref<const Eigen::VectorXd>&,
                          const std::map<std::string, std::string>&,
                          const std::map<std::string, int>&,
                          const std::map<std::string, double>&, int*, double*,
                          EigenPtr<Eigen::VectorXd>) const {
  throw std::runtime_error(
      "The SNOPT bindings were not compiled.  You'll need to use a different "
      "solver.");
}

}  // namespace solvers
}  // namespace drake
