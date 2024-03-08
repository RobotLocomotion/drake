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

/** A version of MakeSemidefiniteRelaxation that allows for specifying the
 * sparsity of the relaxation.
 *
 * For each group in @p variable group, the costs and constraints whose
 * variables are a subset of the group will be jointly relaxed into a single,
 * dense semidefinite program in the same manner as
 * MakeSemidefiniteRelaxation(prog).
 *
 * Each of these semidefinite relaxations are aggregated into a
 * single program, and their semidefinite variables are made to agree where the
 * variable groups overlap.
 *
 * Costs and constraints whose variables are not a subset of any of the groups
 * are not relaxed and are simply added to the aggregated program. If these
 * costs and constraints are non-convex, then this method will throw.
 *
 * @throw std::exception if there is a non-convex cost or constraint whose
 * variables do not intersect with any of the variable groups.
 */
std::unique_ptr<MathematicalProgram> MakeSemidefiniteRelaxation(
    const MathematicalProgram& prog,
    const std::vector<symbolic::Variables>& variable_groups);

}  // namespace solvers
}  // namespace drake
