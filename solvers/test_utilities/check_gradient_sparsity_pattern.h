#pragma once

#include <optional>
#include <unordered_set>
#include <utility>
#include <vector>

#include <Eigen/Dense>
#include <gtest/gtest.h>

#include "drake/common/autodiff.h"
#include "drake/math/autodiff_gradient.h"
#include "drake/solvers/evaluator_base.h"

namespace drake {
namespace solvers {
namespace test {
struct GradientPairHash {
  int cols;

  explicit GradientPairHash(int m_cols) : cols{m_cols} {}

  std::size_t operator()(const std::pair<int, int>& row_col) const {
    return row_col.first * cols + row_col.second;
  }
};

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
template <typename C>
void CheckGradientSparsityPattern(const C& evaluator, const AutoDiffVecXd& x_ad,
                                  bool strict = false) {
  AutoDiffVecXd y_ad(evaluator.num_outputs());
  evaluator.Eval(x_ad, &y_ad);
  const Eigen::MatrixXd y_grad = math::ExtractGradient(y_ad);
  const std::optional<std::vector<std::pair<int, int>>>&
      gradient_sparsity_pattern = evaluator.gradient_sparsity_pattern();
  if (gradient_sparsity_pattern.has_value()) {
    GradientPairHash gradient_tuple_hash(y_grad.cols());
    std::unordered_set<std::pair<int, int>, GradientPairHash>
        gradient_nonzero_indices(gradient_sparsity_pattern->begin(),
                                 gradient_sparsity_pattern->end(),
                                 y_grad.size(), gradient_tuple_hash);
    for (const auto& [row, col] : *gradient_sparsity_pattern) {
      EXPECT_GE(row, 0);
      EXPECT_LT(row, y_grad.rows());
      EXPECT_GE(col, 0);
      EXPECT_LT(col, y_grad.cols());
    }
    for (int i = 0; i < y_grad.rows(); ++i) {
      for (int j = 0; j < y_grad.cols(); ++j) {
        if (y_grad(i, j) != 0) {
          EXPECT_GT(gradient_nonzero_indices.count(std::pair<int, int>(i, j)),
                    0);
        } else if (strict) {
          // y_grad(i, j) == 0
          EXPECT_EQ(gradient_nonzero_indices.count(std::pair<int, int>(i, j)),
                    0);
        }
      }
    }
  }
}
}  // namespace test
}  // namespace solvers
}  // namespace drake
