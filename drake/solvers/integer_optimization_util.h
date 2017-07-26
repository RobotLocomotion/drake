#pragma once

#include "drake/solvers/create_constraint.h"

namespace drake {
namespace solvers {
/**
 * For two binary expressions b1 and b2, each can only take value 0 or 1,
 * impose constraints on b1, b2, together with b1_and_b2, such that
 * b1_and_b2 = b1 ∧ b2.
 * The constraints are
 * <pre>
 *   b1_and_b2 >= b1 + b2 - 1
 *   b1_and_b2 <= b1
 *   b1_and_b2 <= b2
 *   0 <= b1_and_b2 <= 1
 * </pre>
 * @param b1 An expression that should only take binary value.
 * @param b2 An expression that should only take binary value.
 * @param b1_and_b2 Should be the logical and between `b1` and `b2`.
 * @return The newly added constraints, such that when b1, b2, b1_and_b2 satisfy
 * the constraints, it is guaranteed that b1_and_b2 = b1 ∧ b2.
 * @pre b1, b2, b1_and_b2 are all linear expressions.
 */
Binding<LinearConstraint> CreateLogicalAndConstraint(
    const symbolic::Expression& b1, const symbolic::Expression& b2,
    const symbolic::Expression& b1_and_b2);

/**
 * For two binary expressions b1 and b2, each can only take value 0 or 1,
 * impose constraints on b1, b2, together with b1_or_b2, such that
 * b1_or_b2 = b1 ∨ b2.
 * The constraints are
 * <pre>
 *   b1_or_b2 <= b1 + b2
 *   b1_or_b2 >= b1
 *   b1_or_b2 >= b2
 *   0 <= b1_or_b2 <= 1
 * </pre>
 * @param b1 An expression that should only take binary value.
 * @param b2 An expression that should only take binary value.
 * @param b1_or_b2 Should be the logical or between `b1` and `b2`.
 * @return The newly added constraints, such that when b1, b2, b1_or_b2 satisfy
 * the constraints, it is guaranteed that b1_or_b2 = b1 ∨ b2.
 * @pre b1, b2, b1_or_b2 are all linear expressions.
 */
Binding<LinearConstraint> CreateLogicalOrConstraint(const symbolic::Expression& b1, const symbolic::Expression& b2, const symbolic::Expression& b1_or_b2);

/**
 * For two binary expressions b1 and b2, each can only take value 0 or 1,
 * impose constraints on b1, b2, together with b1_xor_b2, such that
 * b1_xor_b2 = b1 ⊕ b2 (b1 exclusive or b2).
 * The constraints are
 * <pre>
 *   b1_xor_b2 <= b1 + b2
 *   b1_xor_b2 >= b1 - b2
 *   b1_xor_b2 >= b2 - b1
 *   b1_xor_b2 <= 2 - b1 - b2
 *   0 <= b1_xor_b2 <= 1
 * </pre>
 * @param b1 An expression that should only take binary value.
 * @param b2 An expression that should only take binary value.
 * @param b1_xor_b2 Should be the logical exclusive or between `b1` and `b2`.
 * @return The newly added constraints, such that when b1, b2, b1_xor_b2 satisfy
 * the constraints, it is guaranteed that b1_xor_b2 = b1 ⊕ b2.
 * @pre b1, b2, b1_xor_b2 are all linear expressions.
 */
Binding<LinearConstraint> CreateLogicalXorConstraint(
    const symbolic::Expression& b1, const symbolic::Expression& b2,
    const symbolic::Expression& b1_xor_b2);
}  // namespace solvers
}  // namespace drake
