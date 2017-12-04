/* clang-format off to disable clang-format-includes */
#include "drake/solvers/nlopt_solver.h"
/* clang-format on */

#include "drake/common/never_destroyed.h"

namespace drake {
namespace solvers {

SolverId NloptSolver::solver_id() const {
  return id();
}

SolverId NloptSolver::id() {
  static const never_destroyed<SolverId> singleton{"NLopt"};
  return singleton.access();
}

}  // namespace solvers
}  // namespace drake
