/* clang-format off to disable clang-format-includes */
#include "drake/solvers/dreal_solver.h"
/* clang-format on */

#include "drake/common/never_destroyed.h"

// This file contains implementations that are common to both the available and
// unavailable flavor of this class.
//
// Note that (for now) _only_ the unavailable flavor of this class exists.
// TODO(jwnimmer-tri) Remove this comment when the available flavor is added.

namespace drake {
namespace solvers {

SolverId DrealSolver::solver_id() const {
  return id();
}

SolverId DrealSolver::id() {
  static const never_destroyed<SolverId> singleton{"dReal"};
  return singleton.access();
}

}  // namespace solvers
}  // namespace drake
