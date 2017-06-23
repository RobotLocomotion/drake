#pragma once

#include <string>

#include "drake/solvers/mathematical_program.h"

namespace drake {
namespace solvers {
/**
 * Return ⌈log₂(n)⌉, namely the minimal integer no smaller than log₂(n), with
 * base 2.
 * @param n A positive integer.
 * @return The minimal integer no smaller than log₂(n).
 */
constexpr int CeilLog2(int n) {
  return n == 1 ? 0 : 1 + CeilLog2((n + 1) / 2);
}

/**
 * Adds the special ordered set 2 (sos2) constraint, that at most two
 * entries in λ can be strictly positive, and these two entries have to be
 * adjacent. All other λ should be zero. Moreover, the non-zero λ satisfies
 * λ(i) + λ(i + 1) = 1.
 * We will need to add ⌈log₂(n - 1)⌉ binary variables, where n is the number of
 * rows in λ. For more information, please refer to
 *   Modeling Disjunctive Constraints with a Logarithmic Number of Binary
 *   Variables and Constraints
 *   by J. Vielma and G. Nemhauser, 2011.
 * @param prog Add the sos2 constraint to this mathematical program.
 * @param lambda At most two entries in λ can be strictly positive, and these
 * two entries have to be adjacent. All other entries are zero.
 * @return y The newly added binary variables. The assignment of the binary
 * variable y implies which two λ can be strictly positive.
 * With a binary assignment on y, and suppose the integer M corresponds to
 * (y(0), y(1), ..., y(⌈log₂(n - 1)⌉)) in Gray code, then only λ(M) and λ(M + 1)
 * can be non-zero. For example, if the assignment of y = (1, 1), in Gray code,
 * (1, 1) represents integer 2, so only λ(2) and λ(3) can be strictly positive.
 */
VectorXDecisionVariable AddLogarithmicSOS2Constraint(
    MathematicalProgram* prog,
    const Eigen::Ref<const VectorX<symbolic::Expression>>& lambda,
    const std::string& binary_variable_name = "y");

/** Adds the special ordered set 2 (sos2) constraint, @see AddLogarithmicSOS2Constraint.
 */
void AddLogarithmicSOS2Constraint(
    MathematicalProgram* prog,
    const Eigen::Ref<const VectorX<symbolic::Expression>>& lambda,
    const Eigen::Ref<const VectorXDecisionVariable>& y);

/**
 * Constrain `w` to approximate the bilinear product x * y. We know
 * that x is in one of the intervals [φx(i), φx(i+1)], y is in one of the
 * intervals [φy(j), φy(j+1)]. The variable `w` is constrained to be in the
 * convex hull of x * y for x in [φx(i), φx(i+1)], y in [φy(j), φy(j+1)], namely
 * (x, y, w) is in the tetrahedron, with vertices [φx(i), φy(j), φx(i)*φy(j)],
 * [φx(i+1), φy(j), φx(i+1)*φy(j)], [φx(i), φy(j+1), φx(i)*φy(j+1)] and
 * [φx(i+1), φy(j+1), φx(i+1)*φy(j+1)]
 * @param prog The program to which the bilinear product constraint is added
 * @param x The decision variable.
 * @param y The decision variable.
 * @param w The expression to approximate x * y
 * @param phi_x The end points of the intervals for `x`.
 * @param phi_y The end points of the intervals for `y`.
 * @param Bx The binary variable, to determine in which interval `x` stays.
 * If Bx represents integer M in Gray code, then `x` is in the interval
 * [φx(M), φx(M+1)].
 * @param By The binary variable, to determine in which interval `y` stays.
 * If Bx represents integer M in Gray code, then `y` is in the interval
 * [φy(M), φy(M+1)].
 */
void AddBilinearProductMcCormickEnvelopeSOS2(
    MathematicalProgram* prog,
    const symbolic::Variable& x,
    const symbolic::Variable& y,
    const symbolic::Expression& w,
    const Eigen::Ref<const Eigen::VectorXd>& phi_x,
    const Eigen::Ref<const Eigen::VectorXd>& phi_y,
    const Eigen::Ref<const VectorXDecisionVariable>& Bx,
    const Eigen::Ref<const VectorXDecisionVariable>& By);
}  // namespace solvers
}  // namespace drake
