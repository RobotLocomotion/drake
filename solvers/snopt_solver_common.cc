/* clang-format off to disable clang-format-includes */
#include "drake/solvers/snopt_solver.h"
/* clang-format on */

#include "drake/common/never_destroyed.h"

namespace drake {
namespace solvers {

SolverId SnoptSolver::solver_id() const {
  return id();
}

SolverId SnoptSolver::id() {
  static const never_destroyed<SolverId> singleton{"SNOPT"};
  return singleton.access();
}

}  // namespace solvers
}  // namespace drake
