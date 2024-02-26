#pragma once

#include <memory>
#include <vector>

#include "drake/solvers/mathematical_program.h"

namespace drake {
namespace solvers {

// TODO(russt): Add an option for using diagonal dominance and/or
// scaled-diagonal dominance instead of the PSD constraint.

// TODO(russt): Consider adding Y as an optional argument return value, to help
// users know the decision variables associated with Y.

/** Constructs a new MathematicalProgram which represents the semidefinite
 programming convex relaxation of the (likely nonconvex) program `prog`. This
 method currently supports only linear and quadratic costs and constraints, but
 may be extended in the future with broader support.

 See https://underactuated.mit.edu/optimization.html#sdp_relaxation for
 references and examples.

 Note: Currently, programs using LinearEqualityConstraint will give tighter
 relaxations than programs using LinearConstraint or BoundingBoxConstraint,
 even if lower_bound == upper_bound. Prefer LinearEqualityConstraint.

 @throws std::exception if `prog` has costs and constraints which are not
 linear nor quadratic.
 */
std::unique_ptr<MathematicalProgram> MakeSemidefiniteRelaxation(
    const MathematicalProgram& prog);

/** Constructs a new MathematicalProgram which represents the semidefinite
 * programming convex relaxation of the (likely nonconvex) program `prog`.
 *
 * For each group in @p variable groups, this method will jointly relax into a
 * semidefinite program all the costs and constraints whose variables intersect
 * with the group. Each of these semidefinite relaxations are aggregated into a
 * single program, and their semidefinite variables are made to agree where they
 * overlap.
 *
 * If a variable group does not intersect any of the costs and constraints, a
 * semidefinite variable is not created for these variables.
 *
 * Costs and constraints whose variables do not overlap with any of the groups
 * are not relaxed and are simply added to the aggregated program. If these
 * costs and constraints are non-convex, then this method will throw.
 *
 * @param prog
 * @param variable_groups
 * @throw Runtime error if there is a non-convex cost or constraint whose
 * variables do not intersect with any of the variable groups.
 * @return
 */
std::unique_ptr<MathematicalProgram> MakeSemidefiniteRelaxation(
    const MathematicalProgram& prog,
    const std::vector<symbolic::Variables>& variable_groups);

}  // namespace solvers
}  // namespace drake
