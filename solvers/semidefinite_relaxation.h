#pragma once

#include <memory>

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

/** Combinations of variables in the original program are "linearized" into
 additional decision variables in the relaxation. For instance, if the original
 `prog` contained the variables, `x` and `y`, and then the relaxed program will
 have `x` and `y`, but also `x²`, `xy`, and `y²` as new decision variables. To
 find the decision variable associated with `xy` in the new program, pass in
 `vars_in_prog = [x, y]`. 

 @pre `relaxation` was constructed by calling `MakeSemidefiniteRelaxation` on
 `prog` and no additional semidefinite constraints have been added.
 @throws std::exception if `vars_in_prog` does not describe a variable in
 `relaxation`.
 */
symbolic::Variable GetVariableInSemidefiniteRelaxation(
    const MathematicalProgram& prog, const MathematicalProgram& relaxation,
    std::vector<symbolic::Variable> vars_in_prog);

}  // namespace solvers
}  // namespace drake
