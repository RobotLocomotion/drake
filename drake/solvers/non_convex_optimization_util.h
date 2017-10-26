#pragma once

#include <utility>

#include <Eigen/Core>

#include "drake/solvers/mathematical_program.h"

namespace drake {
namespace solvers {
/**
 * For a non-convex homogeneous quadratic form xᵀQx, where Q is not necessarily
 * a positive semidefinite matrix, we decompose it as a difference between two
 * convex homogeneous quadratic forms
 * xᵀQx = xᵀQ₁x - xᵀQ₂x,
 * Q₁, Q₂ are positive semidefinite.
 * To find the optimal Q₁ and Q₂, we solve the following semidefinite
 * programming problem
 * min s
 * s.t s >= trace(Q₁)
 *     s >= trace(Q₂)
 *     Q₁ - Q₂ = (Q + Qᵀ) / 2
 *     Q₁, Q₂ are positive semidefinite
 * The decomposition Q = Q₁ - Q₂ can be used later, to solve the non-convex
 * optimization problem involving a quadratic form xᵀQx.
 * For more information, please refer to the papers on difference of convex
 * decomposition, for example
 *   Undominated d.c Decompositions of Quadratic Functions and Applications
 *   to Branch-and-Bound Approaches
 *     By I.M.Bomze and M. Locatelli
 *     Computational Optimization and Applications, 2004
 *   DC Decomposition of Nonconvex Polynomials with Algebraic Techniques
 *     By A. A. Ahmadi and G. Hall
 *     Mathematical Programming, 2015
 * @param Q A square matrix. Throws a runtime_error if Q is not square.
 * @return The optimal decomposition (Q₁, Q₂)
 * TODO(hongkai.dai): templatize this function, to avoid dynamic memory
 * allocation.
 */
std::pair<Eigen::MatrixXd, Eigen::MatrixXd>
DecomposeNonConvexQuadraticForm(const Eigen::Ref<const Eigen::MatrixXd>& Q);

/**
 * For a non-convex quadratic constraint
 *   lb <= xᵀQ₁x - xᵀQ₂x + pᵀx <= ub
 * where Q₁, Q₂ are both positive semidefinite matrices, we relax this
 * constraint by several convex constraints. The steps are
 * 1. Introduce two new variables z₁, z₂, to replace xᵀQ₁x and xᵀQ₂x
 *    respectively. The constraint becomes
 *    <pre>
 *      lb <= z₁ - z₂ + pᵀx <= ub              (1)
 *    </pre>
 * 2. Ideally, we would like to enforce z₁ = xᵀQ₁x and z₂ = xᵀQ₂x through convex
 *    constraints. To this end, we first bound z₁ and z₂ from below, as
 *    <pre>
 *      z₁ >= xᵀQ₁x                            (2)
 *      z₂ >= xᵀQ₂x                            (3)
 *    </pre>
 *    These two constraints are second order cone
 *    constraints.
 * 3. To bound z₁ and z₂ from above, we consider to linearize the quadratic
 *    forms xᵀQ₁x and xᵀQ₂x at a point x₀. Due to the convexity of the quadratic
 *    form, we know that given a positive scalar d, there exists a neighbourhood
 *    around x₀, s.t
 *    <pre>
 *    xᵀQ₁x <= 2 x₀ᵀQ₁(x - x₀) + x₀ᵀQ₁x₀ + d   (4)
 *    xᵀQ₂x <= 2 x₀ᵀQ₂(x - x₀) + x₀ᵀQ₂x₀ + d   (5)
 *    </pre>
 *    so we also enforce the linear constraints
 *    <pre>
 *      z₁ <= 2 x₀ᵀQ₁(x - x₀) + x₀ᵀQ₁x₀ + d    (6)
 *      z₂ <= 2 x₀ᵀQ₂(x - x₀) + x₀ᵀQ₂x₀ + d    (7)
 *    </pre>
 *    So we relax the original non-convex constraint, with the convex
 *    constraints (1)(2)(3)(6)(7)
 *
 * The trust region is the neighbourhood around x₀, such that the inequalities
 * (4), (5) are satisfied.
 *
 * The positive scalar d controls both how much the constraint relaxation is
 * (the original constraint can be violated by at most d), and how big the trust
 * region is.
 *
 * If there is a solution satisfying the relaxed constraint, this solution
 * can violate the original non-convex constraint by at most d; on the other
 * hand, if there is not a solution satisfying the relaxed constraint, it
 * proves that the original non-convex constraint does not have a solution
 * in the trust region.
 *
 * This approach is outlined in section III of
 *   On Time Optimization of Centroidal Momentum Dynamics
 *   by Brahayam Ponton, Alexander Herzog, Stefan Schaal and Ludovic Righetti,
 *   ICRA, 2018
 *
 * The special cases are when Q₁ = 0 or Q₂ = 0.
 * 1. When Q₁ = 0, the constraint becomes
 *    xᵀQ₂x <= pᵀx - lb
 *    xᵀQ₂x >= pᵀx - ub
 *    If ub = +∞, then the original constraint is the convex rotated Lorentz
 *    cone constraint xᵀQ₂x <= pᵀx - lb. The user should not call this function
 *    to relax this convex constraint. Throw a runtime error.
 *    If ub < +∞, then we introduce a new variable z, with the constraints
 *    z >= xᵀQ₂x
 *    z <= 2 x₀ᵀQ₂(x - x₀) + x₀ᵀQ₂x₀ + d
 *    z >= pᵀx - ub
 *    xᵀQ₂x <= pᵀx - lb
 * 2. When Q₂ = 0, the constraint becomes
 *    xᵀQ₁x <= ub - pᵀx
 *    xᵀQ₁x >= lb - pᵀx
 *    If lb = -∞, then the original constraint is the convex rotated Lorentz
 *    cone constraint xᵀQ₁x <= ub - pᵀx. The user should not call this function
 *    to relax this convex constraint. Throw a runtime error.
 *    If lb > -∞, then we introduce a new variable z, with the constraints
 *    z >= xᵀQ₁x
 *    z <= 2 x₀ᵀQ₁(x - x₀) + x₀ᵀQ₁x₀ + d
 *    z >= lb - pᵀx
 *    xᵀQ₁x <= ub - pᵀx
 *
 * @param Q1 A positive definite matrix. Notice we can always add a diagonal
 * matrix a * identity to Q1 and Q2 simultaneously, to make both matrices
 * positive definite, while keeping their difference Q1 - Q2 unchanged.
 * @param Q2 A positive definite matrix.
 * @param p A vector, the linear coefficients in the quadratic form.
 * @param linearization_point The vector `x₀` in the documentation above.
 * @param lower_bound The left-hand side of the original non-convex constraint.
 * @param upper_bound The right-hand side of the original non-convex constraint.
 * @param trust_region_gap The user-specified positive scalar, `d` in
 * the documentation above. This gap determines both the maximal constraint
 * violation, together with the size of the trust region.
 * @retval <linear_constraint, rotated_lorentz_cone1, rotated_lorentz_cone, z>
 * linear_constraint includes (1)(6)(7)
 * rotated_lorentz_cone1 is (2)
 * rotated_lorentz_cone2 is (3)
 * z is the newly added variable.
 * @pre 1. Q1, Q2 are positive semidefinite.
 *      2. d is positive.
 *      3. Q1, Q2, p, x, x₀ are all of the consistent size.
 *      4. lower_bound <= upper_bound.
 */
std::tuple<Binding<LinearConstraint>, Binding<RotatedLorentzConeConstraint>,
           Binding<RotatedLorentzConeConstraint>, VectorDecisionVariable<2>>
RelaxNonConvexQuadraticConstraintInTrustRegion(
    MathematicalProgram* prog,
    const Eigen::Ref<const VectorXDecisionVariable>& x,
    const Eigen::Ref<const Eigen::MatrixXd>& Q1,
    const Eigen::Ref<const Eigen::MatrixXd>& Q2,
    const Eigen::Ref<const Eigen::VectorXd>& p, double lower_bound,
    double upper_bound,
    const Eigen::Ref<const Eigen::VectorXd>& linearization_point,
    double trust_region_gap);
}  // namespace solvers
}  // namespace drake
