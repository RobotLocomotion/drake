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
constexpr int CeilLog2(int n) {
  return n == 1? 0 : 1 + CeilLog2((n + 1) / 2);
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
 * entries in λ can be strictly positive, and these two entries have to be
 * adjacent. All other λ should be zero.
 * We will need to add ⌈log2(n - 1)⌉ binary variables, where n is the number of
 * rows in λ. For
 * more information, please refer to
 *   Modeling Disjunctive Constraints with a Logarithmic Number of Binary
 *   Variables and Constraints
 *   by J. Vielma and G. Nemhauser
 * @param prog
 * @param lambda At most two entries in λ can be non-zero, and these two entries
 * have to be adjacent.
 * @return y The newly added binary variables. The inactivation of the binary
 * variable y implies the inactivation of the expression λ.
 * With an binary assignment on y, and suppose the integer M corresponds to
 * (y(0), y(1), ..., y(⌈log2(n - 1)⌉)) in Gray code, then only λ(M) and λ(M + 1)
 * can be non-zero.
 */
VectorXDecisionVariable
AddLogarithmicSOS2Constraint(MathematicalProgram* prog, const Eigen::Ref<const VectorX<symbolic::Expression>>& lambda, const std::string& binary_variable_name = "b");
}  // namespace solvers
}  // namespace drake