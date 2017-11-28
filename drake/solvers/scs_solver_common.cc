/* clang-format off to disable clang-format-includes */
#include "drake/solvers/scs_solver.h"
/* clang-format on */

#include "drake/common/never_destroyed.h"

namespace drake {
namespace solvers {

SolverId ScsSolver::solver_id() const {
  return id();
}

SolverId ScsSolver::id() {
  static const never_destroyed<SolverId> singleton{"SCS"};
  return singleton.access();
}

}  // namespace solvers
}  // namespace drake
