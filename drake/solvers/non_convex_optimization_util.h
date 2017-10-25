#pragma once

#include <utility>

#include <Eigen/Core>

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
}  // namespace solvers
}  // namespace drake
