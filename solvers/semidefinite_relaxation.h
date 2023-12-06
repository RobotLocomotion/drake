#pragma once

#include <memory>
#include <optional>

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

 The optional parameter variable_groups can be used to make a cheaper relaxation
 with fewer linear constraints. Specifically, only linear constraints whose
 variables intersect the same entry in variable_groups will be multiplied. For
 example, if there are three linear constraints Ax ≥ 0, By ≥ 0, Cz ≥ 0, and
 variables_groups = [{x,y}, {y,z}], then the semidefinite relaxation will
 include the linear constraints AxyᵀB ≥ 0 and ByzᵀC ≥ 0, but not AxzᵀC ≥ 0. If
 variable_groups is an empty vector, then no linear constraints will be
 multiplied. If variable_groups is nullopt, then all linear constraints will be
 multiplied.

 @throws std::exception if `prog` has costs and constraints which are not
 linear nor quadratic.
 */
std::unique_ptr<MathematicalProgram> MakeSemidefiniteRelaxation(
    const MathematicalProgram& prog,
//    const std::optional<std::vector<symbolic::Variables>> variable_groups =
//        std::nullopt
        );

}  // namespace solvers
}  // namespace drake
