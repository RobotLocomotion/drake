/* clang-format off to disable clang-format-includes */
#include "drake/solvers/mosek_solver.h"
/* clang-format on */

#include "drake/common/never_destroyed.h"

namespace drake {
namespace solvers {

SolverId MosekSolver::solver_id() const {
  return id();
}

SolverId MosekSolver::id() {
  static const never_destroyed<SolverId> singleton{"Mosek"};
  return singleton.access();
}

}  // namespace solvers
}  // namespace drake
