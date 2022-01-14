#pragma once

#include <vector>

#include "drake/solvers/mathematical_program.h"

namespace drake {
namespace solvers {
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
 * @param[out] constraint_residue The value of the all the constraints. For an
 * equality constraint c(x)=0 or the inequality constraint c(x)>= 0, the residue
 * is c(x). Depending on include_x_bounds, `constraint_residue` may or may not
 * contain the residue for bounding box constraints x_lb <= x <= x_ub at the
 * end.
 * @param[out] cost The value of the cost function f(x).
 * @return al The evaluated Augmented Lagrangian (AL) L(x, λ, μ)
 */
template <typename T>
T EvalAugmentedLagrangian(const MathematicalProgram& prog,
                          const Eigen::Ref<const VectorX<T>>& x,
                          const Eigen::VectorXd& lambda, double mu,
                          bool include_x_bounds, VectorX<T>* constraint_residue,
                          T* cost);

/**
 * Returns the size of the lagrangian multiplier for the augmented Lagrangian
 * (AL) method. For the definition of lagrangian multipliers, refer to
 * EvalAugmentedLagrangian.
 * @param prog The optimization program whose augmented Lagrangian will be
 * computed.
 * @param include_x_bounds Whether the lagrangian multipliers include those for
 * the constraints x_lo <= x <= x_up.
 */
[[nodiscard]] int GetLagrangianSizeForAl(const MathematicalProgram& prog,
                                         bool include_x_bounds);

/**
 * Get the type (equality/inequality) of the constraint for augmented Lagrangian
 * (AL). Refer to EvalAugmentedLagrangian for more details.
 * @param include_x_bounds If set to true, then we also consider the type of the
 * constraint x_lo <= x <= x_up at the end of @p is_equality.
 * @return is_equality is_equality will have the same size as
 * GetLagragrangianSizeForAl. We consider all the constraints in the order of
 * prog.GetAllConstraints() (except for the bounding box constraints which we
 * will consider in the end if include_x_bounds=true). For a constraint lo <=
 * h(x) <= up we consider the lower bound first and then the upper bound.
 */
std::vector<bool> IsEqualityForAl(const MathematicalProgram& prog,
                                  bool include_x_bounds);
}  // namespace solvers
}  // namespace drake
