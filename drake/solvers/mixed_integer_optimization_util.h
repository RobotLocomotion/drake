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
}  // namespace solvers
}  // namespace drake
