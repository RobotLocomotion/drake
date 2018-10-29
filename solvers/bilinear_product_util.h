#pragma once

#include "drake/common/eigen_types.h"
#include "drake/common/symbolic.h"
#include "drake/solvers/decision_variable.h"

namespace drake {
namespace solvers {
/**
  * Replaces all the bilinear product terms in the expression `e`, with the
 * corresponding terms in `W`, where `W` represents the matrix x * yᵀ, such that
 * after replacement, `e` does not have bilinear terms involving `x` and `y`.
 * For example, if e = x(0)*y(0) + 2 * x(0)*y(1) + x(1) * y(1) + 3 * x(1), `e`
 * has bilinear terms x(0)*y(0), x(0) * y(1) and x(2) * y(1), if we call
 * ReplaceBilinearTerms(e, x, y, W) where W(i, j) represent the term x(i) *
 * y(j), then this function returns W(0, 0) + 2 * W(0, 1) + W(1, 1) + 3 * x(1).
 * @param e An expression potentially contains bilinear products between x and
 * y.
 * @param x The bilinear product between `x` and `y` will be replaced by the
 * corresponding term in `W`.
 * @throws std::runtime_error if `x` contains duplicate entries.
 * @param y The bilinear product between `x` and `y` will be replaced by the
 * corresponding term in `W.
 * @throws std::runtime_error if `y` contains duplicate entries.
 * @param W Bilinear product term x(i) * y(j) will be replaced by W(i, j). If
 * W(i,j) is not a single variable, but an expression, then this expression
 * cannot contain a variable in either x or y.
 * @throws std::runtime_error, if W(i, j) is not a single variable, and also
 * contains a variable in x or y.
 * @return The symbolic expression after replacing x(i) * y(j) with W(i, j).
 */
symbolic::Expression ReplaceBilinearTerms(
    const symbolic::Expression& e,
    const Eigen::Ref<const VectorXDecisionVariable>& x,
    const Eigen::Ref<const VectorXDecisionVariable>& y,
    const Eigen::Ref<const MatrixX<symbolic::Expression>>& W);
}  // namespace solvers
}  // namespace drake
