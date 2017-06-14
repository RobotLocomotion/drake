#pragma once

#include "drake/solvers/mathematical_program.h"

namespace drake {
namespace solvers {
/**
 * Return ⌈log(n)⌉, namely the minimal integer no smaller than log2(n), with
 * base 2.
 * @param n A positive integer.
 * @return The minimal integer no smaller than log2(n).
 */
constexpr int ceil_log2(int n) {
  return n == 1? 0 : 1 + ceil_log2((n + 1) / 2);
}



namespace internal {
/**
 * Return the k-digit reflected Gray codes.
 * @param k The number of digits in the Gray code.
 * @retval m m.row(i) is the Gray code for integer i.
 */
Eigen::MatrixXi CalculateReflectedGrayCodes(int k);

/**
 * Convert the Gray code to an integer.
 * (0, 0) -> 0
 * (0, 1) -> 1
 * (1, 1) -> 2
 * (1, 0) -> 3
 * @param grey_code
 * @return
 */
int GrayCodeToInteger(const Eigen::Ref<const Eigen::VectorXi>& gray_code);
}  // namespace internal

/**
 * Adds the special ordered set 2 (SOS2) constraint, that at most two
 * consecutive entries in lambda can be non-zero. We will need to add
 * ⌈log2(n - 1)⌉ binary variables, where n is the number of rows in lambda. For
 * more information, please refer to
 *   Modeling Disjunctive Constraints with a Logarithmic Number of Binary
 *   Variables and Constraints
 *   by J. Vielma and G. Nemhauser
 * @param prog
 * @param lambda
 * @return The newly added binary variables, and the linear constraints.
 * To recover which two λ(i) and λ(i+1) are non-zero from the binary variables,
 * first convert the binary variables to an integer \p M using Gray code, the
 * non-zero λ is λ(M) and λ(M + 1)
 */
std::pair<VectorXDecisionVariable, Binding<LinearConstraint>>
AddLogarithmicSOS2Constraint(MathematicalProgram* prog, const Eigen::Ref<const VectorX<symbolic::Expression>>& lambda);
}  // namespace solvers
}  // namespace drake