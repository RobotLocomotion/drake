#pragma once

#include <memory>
#include <vector>

#include "drake/solvers/mathematical_program.h"

namespace drake {
namespace solvers {

// TODO(russt): Add an option for using diagonal dominance and/or
// scaled-diagonal dominance instead of the PSD constraint.

/** Configuration options for the MakeSemidefiniteRelaxation. Throughout these
 * options, we refer to the variables of the original optimization program as y,
 * and the semidefinite variable of the associate relaxation as X.
 *
 * X has the structure
 * X = [Y,    y]
 *     [yᵀ, one]
 */
struct SemidefiniteRelaxationOptions {
  /** Given a program with the linear constraints Ay ≤ b, sets whether to add
   * the implied linear constraints [A,-b]X[A,-b]ᵀ ≤ 0 to the semidefinite
   * relaxation.*/
  bool add_implied_linear_equality_constraints{true};

  /** Given a program with the linear equality constraints Ay = b, sets whether
   * to add the implied linear constraints [A, -b]X = 0 to the semidefinite
   * relaxation.*/
  bool add_implied_linear_constraints{true};

  /** Configure the semidefinite relaxation options to provide the strongest
   * possible semidefinite relaxation that we currently support. This in general
   * will give the tightest convex relaxation we support, but the longest solve
   * times.*/
  void set_to_strongest() {
    add_implied_linear_equality_constraints = true;
    add_implied_linear_constraints = true;
  }

  /** Configure the semidefinite relaxation options to provide the weakest
   * semidefinite relaxation that we currently support. This in general will
   * create the loosest convex relaxation we support, but the shortest solve
   * times. This is equivalent to the standard Shor Relaxation (see Quadratic
   * Optimization Problems by NZ Shor or Semidefinite Programming by
   * Vandenberghe and Boyd). */
  void set_to_weakest() {
    add_implied_linear_equality_constraints = false;
    add_implied_linear_constraints = false;
  }
};

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
    const MathematicalProgram& prog,
    const SemidefiniteRelaxationOptions& options = {});

/** A version of MakeSemidefiniteRelaxation that allows for specifying the
 * sparsity of the relaxation.
 *
 * For each group in @p variable_groups, the costs and constraints whose
 * variables are a subset of the group will be jointly relaxed into a single,
 * dense semidefinite program in the same manner as
 * MakeSemidefiniteRelaxation(prog).
 *
 * Each of these semidefinite relaxations are aggregated into a
 * single program, and their semidefinite variables are made to agree where the
 * variable groups overlap.
 *
 * The returned program will always have the same number of PSD variables as
 * variable groups.
 *
 * Costs and constraints whose variables are not a subset of any of the groups
 * are not relaxed and are simply added to the aggregated program. If these
 * costs and constraints are non-convex, then this method will throw.
 *
 * As an example, consider the following program.
 * min x₂ᵀ * Q * x₂ subject to
 * x₁ + x₂ ≤ 1
 * x₂ + x₃ ≤ 2
 * x₁ + x₃ ≤ 3
 *
 * And suppose we call
 * MakeSemidefiniteRelaxation(prog, std::vector<Variables>{{x₁, x₂}, {x₂,x₃}}).
 *
 * The resulting relaxation would have two semidefinite variables, namely:
 * [U₁,  U₂,  x₁]   [W₁,  W₂, x₂]
 * [U₂,  U₃,  x₂],  [W₂,  W₃, x₃]
 * [x₁ᵀ, x₂ᵀ,  1]   [x₂ᵀ, x₃ᵀ, 1]
 *
 * The first semidefinite variable would be associated to the semidefinite
 * relaxation of the subprogram:
 * min x₁ᵀ * Q * x₁ subject to
 * x₁ + x₂ ≤ 1
 * And the implied constraints from x₁ + x₂ ≤ 1 would be added to the first
 * semidefinite variable. These implied constraints are additional constraints
 * that can be placed on the matrix
 * [U₁,  U₂,  x₁]
 * [U₂,  U₃,  x₂]
 * [x₁ᵀ, x₂ᵀ,  1]
 * which are redundant in the non-convex program, but are not redundant in the
 * semidefinite relaxation. See
 * https://underactuated.mit.edu/optimization.html#sdp_relaxation for references
 * and examples.
 *
 * The second semidefinite variable would be associated to the semidefinite
 * relaxation of the subprogram:
 * min x₂ᵀ * Q * x₂ subject to
 * x₂ + x₃ ≤ 2
 * And the implied constraints from x₂ + x₃ ≤ 2 would be added to the second
 * semidefinite variable.
 *
 * Since the constraint x₁ + x₃ ≤ 3 is not a subset of any of the variable
 * groups, it will be added to the overall relaxation, but will not be used to
 * generate implied constraints on any semidefinite variable.
 *
 * The total relaxation would also include an equality constraint that U₃ == W₁
 * so that the quadratic relaxation of x₂ is consistent between the two
 * semidefinite variables.
 *
 * Note:
 * 1) Costs are only associated to a single variable group, so that the
 * resulting aggregated program has a relaxed cost with the same scaling.
 * 2) The homogenization variable "1" is re-used in every semidefinite variable.
 *
 * @throw std::exception if there is a non-convex cost or constraint whose
 * variables do not intersect with any of the variable groups.
 */
std::unique_ptr<MathematicalProgram> MakeSemidefiniteRelaxation(
    const MathematicalProgram& prog,
    const std::vector<symbolic::Variables>& variable_groups,
    const SemidefiniteRelaxationOptions& options = {});

}  // namespace solvers
}  // namespace drake
