#pragma once

#include <utility>

#include <Eigen/Core>

#include "drake/solvers/constraint.h"

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
 * For a non-convex quadratic inequality constraint
 *   xᵀQ₁x - xᵀQ₂x + pᵀx <= r
 * where Q₁, Q₂ are both positive semidefinite matrices, we relax this
 * constraint by several convex constraints. The steps are
 * 1. Linearize xᵀQ₂x at a point x₀, the linear function
 *    2 x₀ᵀQ₂(x - x₀) + x₀ᵀQ₂x₀ is a lower bound of xᵀQ₂x, due to the convexity.
 * 2. Shift the linear function uppward by d, as
 *    2 x₀ᵀQ₂(x - x₀) + x₀ᵀQ₂x₀ + d, where d is a positive scalar. Within a
 *    neighbourhood of the linearization point x₀, this linear function is an
 *    upper bound of the quadratic form xᵀQ₂x, the maximal error between the
 *    linear upper bound and the quadratic form is d.
 * 3. Relax the original constraint as
 *    xᵀQ₁x <= r + xᵀQ₂x - pᵀx
 *          <= r + 2 x₀ᵀQ₂(x - x₀) + x₀ᵀQ₂x₀ + d - pᵀx
 *    This is a convex second order cone constraint.
 *    If there is a solution satisfying the relaxed constraint, this solution
 *    can violate the original non-convex constraint by at most d; on the other
 *    hand, if there is not a solution satisfying the relaxed constraint, it
 *    proves that the original non-convex constraint does not have a solution
 *    in the neighbourhood of x₀, where the neighbourhood is defined as
 *    {x | 2 x₀ᵀQ₂(x - x₀) + x₀ᵀQ₂x₀ + d >= xᵀQ₂x}. This neighbourhood is the
 *    trust-region of this approach.
 * This approach is outlined in section III of
 *   On Time Optimization of Centroidal Momentum Dynamics
 *   by Brahayam Ponton, Alexander Herzog, Stefan Schaal and Ludovic Righetti,
 *   ICRA, 2018
 * @param Q1 A positive definite matrix. Notice we can always add a diagonal
 * matrix a * identity to Q1 and Q2 simultaneously, to make both matrices
 * positive definite, while keeping their difference Q1 - Q2 unchanged.
 * @param Q2 A positive definite matrix.
 * @param p A vector, the linear coefficients in the quadratic form.
 * @param linearization_point The vector `x₀` in the documentation above.
 * @param upper_bound The right-hand side of the inequality constraint, `r` in
 * the documentation above.
 * @param trust_region_gap The user-specified positive scalar, `d` in
 * the documentation above. This gap determines both the maximal constraint
 * violation, together with the size of the trust region.
 * @return The newly constructed rotated Lorentz cone constraint.
 * @pre 1. Q1, Q2 are positive definite.
 *      2. d is positive.
 *      3. Q1, Q2, p, x₀ are all of the consistent size.
 */
std::shared_ptr<RotatedLorentzConeConstraint>
RelaxNonConvexQuadraticInequalityConstraintInTrustRegion(
    const Eigen::Ref<const Eigen::MatrixXd>& Q1,
    const Eigen::Ref<const Eigen::MatrixXd>& Q2,
    const Eigen::Ref<const Eigen::VectorXd>& p, double upper_bound,
    const Eigen::Ref<const Eigen::VectorXd>& linearization_point,
    double trust_region_gap);
}  // namespace solvers
}  // namespace drake
