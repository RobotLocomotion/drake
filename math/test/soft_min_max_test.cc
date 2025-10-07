#include "drake/math/soft_min_max.h"

#include <vector>

#include <gtest/gtest.h>

#include "drake/common/autodiff.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"

namespace drake {
namespace math {
GTEST_TEST(SoftOverMax, TestDouble) {
  const std::vector<double> x{{1.0, 2.0, 3.0, 1.5, 2.9, 2.99}};
  double alpha = 1;
  const double x_max1 = SoftOverMax(x, alpha);
  EXPECT_GT(x_max1, 3);

  alpha = 10;
  const double x_max10 = SoftOverMax(x, alpha);
  EXPECT_GT(x_max10, 3);
  EXPECT_LT(x_max10, x_max1);
  EXPECT_NEAR(x_max10, 3, 0.1);

  alpha = 100;
  const double x_max100 = SoftOverMax(x, alpha);
  EXPECT_GT(x_max100, 3);
  EXPECT_LT(x_max100, x_max10);
  EXPECT_NEAR(x_max100, 3, 0.01);
}

GTEST_TEST(SoftOverMax, TestAutodiff) {
  std::vector<AutoDiffXd> x;
  x.emplace_back(1, Eigen::Vector2d(1, 3));
  x.emplace_back(2, Eigen::Vector2d(0, 1));
  x.emplace_back(3, Eigen::Vector2d(0, -1));
  const AutoDiffXd x_max = SoftOverMax(x, 10);
  EXPECT_GT(x_max.value(), 3);
  EXPECT_NEAR(x_max.value(), 3, 0.01);
  EXPECT_TRUE(
      CompareMatrices(x_max.derivatives(), Eigen::Vector2d(0, -1), 1E-3));
}

GTEST_TEST(SoftUnderMax, TestDouble) {
  const std::vector<double> x{{0, 1, 2, 3, 0.5, 2.5, 2.9}};
  double alpha = 1;
  const double x_max1 = SoftUnderMax(x, alpha);
  EXPECT_LT(x_max1, 3);

  alpha = 10;
  const double x_max10 = SoftUnderMax(x, alpha);
  EXPECT_LT(x_max10, 3);
  EXPECT_LT(x_max1, x_max10);
  EXPECT_NEAR(x_max10, 3, 0.1);

  alpha = 100;
  const double x_max100 = SoftUnderMax(x, alpha);
  EXPECT_LT(x_max100, 3);
  EXPECT_LT(x_max10, x_max100);
  EXPECT_NEAR(x_max100, 3, 1E-5);
}

GTEST_TEST(SoftUnderMax, TestAutoDiff) {
  std::vector<AutoDiffXd> x;
  x.emplace_back(0, Eigen::Vector2d(3, 4));
  x.emplace_back(1, Eigen::Vector2d(1, 2));
  x.emplace_back(2, Eigen::Vector2d(0, -1));
  const AutoDiffXd x_max = SoftUnderMax(x, 10 /* alpha */);
  EXPECT_LT(x_max.value(), 2);
  EXPECT_NEAR(x_max.value(), 2, 1E-2);
  EXPECT_TRUE(
      CompareMatrices(x_max.derivatives(), Eigen::Vector2d(0, -1), 1E-2));
}

GTEST_TEST(SoftOverMin, TestDouble) {
  const std::vector<double> x{{1, 1.2, 1.5, 2, 3, 4}};
  double alpha = 1;
  const double x_min1 = SoftOverMin(x, alpha);
  EXPECT_GT(x_min1, 1);

  alpha = 10;
  const double x_min10 = SoftOverMin(x, alpha);
  EXPECT_GT(x_min10, 1);
  EXPECT_LT(x_min10, x_min1);
  EXPECT_NEAR(x_min10, 1, 0.1);

  alpha = 100;
  const double x_min100 = SoftOverMin(x, alpha);
  EXPECT_GT(x_min100, 1);
  EXPECT_LT(x_min100, x_min10);
  EXPECT_NEAR(x_min100, 1, 1E-3);
}

GTEST_TEST(SoftOverMin, TestAutoDiff) {
  std::vector<AutoDiffXd> x;
  x.emplace_back(0, Eigen::Vector2d(3, 4));
  x.emplace_back(1, Eigen::Vector2d(1, 2));
  x.emplace_back(2, Eigen::Vector2d(0, -1));
  const AutoDiffXd x_min = SoftOverMin(x, 10 /* alpha */);
  EXPECT_GT(x_min.value(), 0);
  EXPECT_NEAR(x_min.value(), 0, 1E-2);
  EXPECT_TRUE(
      CompareMatrices(x_min.derivatives(), Eigen::Vector2d(3, 4), 1E-2));
}

GTEST_TEST(SoftUnderMin, TestDouble) {
  std::vector<double> x{{1, 1.2, 1.5, 2, 3, 4, 5}};
  double alpha = 1;
  const double x_min1 = SoftUnderMin(x, alpha);
  EXPECT_LT(x_min1, 1);

  alpha = 10;
  const double x_min10 = SoftUnderMin(x, alpha);
  EXPECT_LT(x_min10, 1);
  EXPECT_GT(x_min10, x_min1);
  EXPECT_NEAR(x_min10, 1, 0.1);

  alpha = 100;
  const double x_min100 = SoftUnderMin(x, alpha);
  EXPECT_LT(x_min100, 1);
  EXPECT_GT(x_min100, x_min10);
  EXPECT_NEAR(x_min100, 1, 1E-3);
}

GTEST_TEST(SoftUnderMin, TestAutoDiff) {
  std::vector<AutoDiffXd> x;
  x.emplace_back(0, Eigen::Vector2d(3, 4));
  x.emplace_back(1, Eigen::Vector2d(1, 2));
  x.emplace_back(2, Eigen::Vector2d(0, -1));
  const AutoDiffXd x_min = SoftUnderMin(x, 10 /* alpha */);
  EXPECT_LT(x_min.value(), 0);
  EXPECT_NEAR(x_min.value(), 0, 1E-2);
  EXPECT_TRUE(
      CompareMatrices(x_min.derivatives(), Eigen::Vector2d(3, 4), 1E-2));
}
}  // namespace math
}  // namespace drake
