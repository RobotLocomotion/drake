#pragma once

#include "drake/solvers/mathematical_program.h"

namespace drake {
namespace solvers {
namespace internal {
/**
 * Compute the augmented lagrangian of a given mathematical program
 *
 *     min f(x)
 *     s.t l <= g(x) <= u
 *         x_lo <= x <= x_up
 *
 * We first turn it into an equality constrained program with non-negative slack
 * variable s as follows
 *
 *     min f(x)
 *     s.t h(x) = 0
 *         c(x) - s = 0
 *         s >= 0
 *
 * Depending on the option include_x_bounds, the constraint h(x)=0, c(x)>=0 may
 * or may not include the bounding box constraint x_lo <= x <= x_up.
 *
 * the augmented Lagrangian is defined as
 *
 *     L(x, λ, μ) = f(x) − λ₁ᵀh(x) + 1/(2μ) h(x)ᵀh(x)
 *                  - λ₂ᵀ(c(x)-s) + 1/(2μ) (c(x)-s)ᵀ(c(x)-s)
 *
 * where s = max(c(x) - μλ₂, 0).
 *
 * For more details, refer to section 17.4 of Numerical Optimization by Jorge
 * Nocedal and Stephen Wright.
 * @param prog The mathematical program we will evaluate.
 * @param x The value of all the decision variables.
 * @param lambda The estimated Lagrangian multipliers. The order of the
 * Lagrangian multiplier is as this: We first call to evaluate all
 * constraints. Then for each row of the constraint, if it is an equality
 * constraint, then we append one single Lagrangian multiplier. Otherwise we
 * append the Lagrangian multipler for the lower and upper bounds, if each
 * corresponding bound is not ±∞. The order of evaluating all the constraints is
 * the same as prog.GetAllConstraints() except for
 * prog.bounding_box_constraints(). If include_x_bounds=true, then we aggregate
 * all the bounding_box_constraints() and evaluate them at the end of all
 * constraints.
 * @param mu μ in the documentation above. The constant for penalty term weight.
 * This should be a strictly positive number.
 * @param include_x_bounds. Whether the Lagrangian and the penalty for the
 * bounds x_lo <= x <= x_up are included in the augmented Lagrangian L(x, λ, μ)
 * or not.
 * @return al The evaluated Augmented Lagrangian (AL) L(x, λ, μ)
 * @note Currently we consider the Lagrangian multiplier and penalty for all
 * constraints, including the bounding box constraint x_lower <= x <= x_upper.
 */
template <typename T>
T EvalAugmentedLagrangian(const MathematicalProgram& prog,
                          const Eigen::Ref<const VectorX<T>>& x,
                          const Eigen::VectorXd& lambda, double mu,
                          bool include_x_bounds);
}  // namespace internal
}  // namespace solvers
}  // namespace drake
