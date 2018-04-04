#pragma once

#include "drake/solvers/binding.h"
#include "drake/solvers/constraint.h"

namespace drake {
namespace solvers {
/**
 * Adds linear constraints, such that when b1, b2, b1_and_b2 satisfy the
 * constraints, and b1, b2 take binary values, it is guaranteed that
 * b1_and_b2 = b1 ∧ b2 (b1 and b2).
 * The constraints are
 * <pre>
 *   b1_and_b2 >= b1 + b2 - 1
 *   b1_and_b2 <= b1
 *   b1_and_b2 <= b2
 *   0 <= b1_and_b2 <= 1
 * </pre>
 * @param b1 An expression that should only take a binary value.
 * @param b2 An expression that should only take a binary value.
 * @param b1_and_b2 Should be the logical and between `b1` and `b2`.
 * @return The newly added constraints, such that when b1, b2, b1_and_b2 satisfy
 * the constraints, it is guaranteed that b1_and_b2 = b1 ∧ b2.
 * @pre b1, b2, b1_and_b2 are all linear expressions.
 */
Binding<LinearConstraint> CreateLogicalAndConstraint(
    const symbolic::Expression& b1, const symbolic::Expression& b2,
    const symbolic::Expression& b1_and_b2);

/**
 * Adds linear constraints, such that when b1, b2, b1_or_b2 satisfy the
 * constraints, and b1, b2 take binary values, it is guaranteed that
 * b1_or_b2 = b1 ∨ b2 (b1 or b2).
 * The constraints are
 * <pre>
 *   b1_or_b2 <= b1 + b2
 *   b1_or_b2 >= b1
 *   b1_or_b2 >= b2
 *   0 <= b1_or_b2 <= 1
 * </pre>
 * @param b1 An expression that should only take a binary value.
 * @param b2 An expression that should only take a binary value.
 * @param b1_or_b2 Should be the logical or between `b1` and `b2`.
 * @return The newly added constraints, such that when b1, b2, b1_or_b2 satisfy
 * the constraints, it is guaranteed that b1_or_b2 = b1 ∨ b2.
 * @pre b1, b2, b1_or_b2 are all linear expressions.
 */
Binding<LinearConstraint> CreateLogicalOrConstraint(
    const symbolic::Expression& b1, const symbolic::Expression& b2,
    const symbolic::Expression& b1_or_b2);

/**
 * Add linear constraints, such that when b1, b2, b1_xor_b2 satisfy the
 * constraints, and b1, b2 take binary values, it is guaranteed that
 * b1_xor_b2 = b1 ⊕ b2 (b1 exclusive xor b2).
 * The constraints are
 * <pre>
 *   b1_xor_b2 <= b1 + b2
 *   b1_xor_b2 >= b1 - b2
 *   b1_xor_b2 >= b2 - b1
 *   b1_xor_b2 <= 2 - b1 - b2
 *   0 <= b1_xor_b2 <= 1
 * </pre>
 * @param b1 An expression that should only take a binary value.
 * @param b2 An expression that should only take a binary value.
 * @param b1_xor_b2 Should be the logical exclusive or between `b1` and `b2`.
 * @return The newly added constraints, such that when b1, b2, b1_xor_b2 satisfy
 * the constraints, it is guaranteed that b1_xor_b2 = b1 ⊕ b2.
 * @pre b1, b2, b1_xor_b2 are all linear expressions.
 */
Binding<LinearConstraint> CreateLogicalXorConstraint(
    const symbolic::Expression& b1, const symbolic::Expression& b2,
    const symbolic::Expression& b1_xor_b2);

/** Create linear constraints such that, when these constraints are satisfied,
 * match = 1 if and only if code == expected, otherwise match = 0
 * @param code code(i) should only take binary values.
 * @param expected The expected matched value for code.
 * @param match an expression that takes binary value, representing if
 * code == expected
 * @return the linear constraints.
 *
 * This function is useful integer optimization, for example, if we have a
 * constraint match = ((b1 == 0) && (b2 == 1) && (b3 == 1)), we can call the
 * function CreateBinaryCodeMatchConstraint({b1, b2, b3}, {0, 1, 1}, match) to
 * create the constraint.
 */
Binding<LinearConstraint> CreateBinaryCodeMatchConstraint(
    const VectorX<symbolic::Expression>& code,
    const Eigen::Ref<const Eigen::VectorXi>& expected,
    const symbolic::Expression& match);
}  // namespace solvers
}  // namespace drake
