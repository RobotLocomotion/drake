#pragma once

#include <Eigen/Dense>

#include "drake/common/autodiff.h"
#include "drake/solvers/evaluator_base.h"

namespace drake {
namespace solvers {
namespace test {
/**
 * Checks the gradient sparsity pattern, by evaluating `evaluator` at `x_ad`,
 * and compare whether the gradient matches with
 * evaluator.gradient_sparsity_pattern(). Notice that
 * `evaluator.gradients_sparsity_pattern()` contains all the (row, col) indices
 * where the gradient **might** be non-zero, so
 * `evaluator.gradients_sparsity_pattern()` is a superset of all the non-zero
 * gradient indices. If `strict`=true, then we check if
 * `evaluator.gradients_sparsity_pattern()` is exactly the set of all the
 * non-zero gradient_indices.
 */
void CheckGradientSparsityPattern(const EvaluatorBase& evaluator,
                                  const AutoDiffVecXd& x_ad,
                                  bool strict = false);
}  // namespace test
}  // namespace solvers
}  // namespace drake
