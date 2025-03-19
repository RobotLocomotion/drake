#include "drake/solvers/test_utilities/check_gradient_sparsity_pattern.h"

#include <optional>
#include <unordered_set>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "drake/math/autodiff_gradient.h"
namespace drake {
namespace solvers {
namespace test {

void CheckGradientSparsityPattern(const EvaluatorBase& evaluator,
                                  const AutoDiffVecXd& x_ad, bool strict) {
  AutoDiffVecXd y_ad(evaluator.num_outputs());
  evaluator.Eval(x_ad, &y_ad);
  const Eigen::MatrixXd y_grad = math::ExtractGradient(y_ad);
  const std::optional<std::vector<std::pair<int, int>>>&
      gradient_sparsity_pattern = evaluator.gradient_sparsity_pattern();
  if (gradient_sparsity_pattern.has_value()) {
    std::unordered_set<std::pair<int, int>, DefaultHash>
        gradient_nonzero_indices(gradient_sparsity_pattern->begin(),
                                 gradient_sparsity_pattern->end(),
                                 y_grad.size());
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
