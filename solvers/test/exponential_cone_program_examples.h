#pragma once

#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/solver_interface.h"

namespace drake {
namespace solvers {
namespace test {
/**
 * Solves a trivial problem with exponential cone constraint
 * min y*exp(z/y)
 * s.t y + z = 1
 *     y > 0
 * @param check_dual If set to true, we will also check the dual solution.
 */
void ExponentialConeTrivialExample(const SolverInterface& solver, double tol,
                                   bool check_dual);

/**
 * For a random variable x (assuming that the sample space of x is  {0, 1, 2,
 * 3}), with a given probability p(x), find the other probability q(x), such
 * that the KL divergence KL(p(x) | q(x)) is minimized. The optimal probability
 * is q(x) = p(x).
 */
void MinimizeKLDivergence(const SolverInterface& solver, double tol);

/**
 * Given several points, find the smallest ellipsoid that covers these points.
 * Mathematically, this problem can be formulated as
 * max log(det(S))
 * s.t. ⌈S     b/2⌉ is positive semidifinite.
 *      ⌊bᵀ/2  c  ⌋
 *     pᵀ * S * p + bᵀ * p + c <= 1 for all p.
 * where the ellipsoid is described as {x | xᵀ*S*x  + bᵀ*x + c <= 1}, and p is
 * a point to be covered.
 */
void MinimalEllipsoidCoveringPoints(const SolverInterface& solver, double tol);

}  // namespace test
}  // namespace solvers
}  // namespace drake
