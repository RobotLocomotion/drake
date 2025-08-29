#include "drake/math/differentiable_norm.h"

#include <limits>

#include <gtest/gtest.h>

#include "drake/common/autodiff.h"
#include "drake/common/eigen_types.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"

namespace drake {
namespace math {
namespace {
const double kEps = std::numeric_limits<double>::epsilon();

GTEST_TEST(TestDifferentiableNorm, test_double) {
  EXPECT_NEAR(DifferentiableNorm(Eigen::Vector3d(1, 0, 0)), 1., kEps);
}

GTEST_TEST(TestDifferentiableNorm, test_autodiff) {
  Eigen::Matrix3Xd x_grad(3, 1);
  x_grad << 1, 2, 3;
  Vector3<AutoDiffXd> x = InitializeAutoDiff(Eigen::Vector3d(1, 0, 0), x_grad);
  AutoDiffXd norm = DifferentiableNorm(x);
  AutoDiffXd norm_expected = x.norm();
  EXPECT_NEAR(norm.value(), norm_expected.value(), 10 * kEps);
  EXPECT_TRUE(CompareMatrices(norm.derivatives(), norm_expected.derivatives(),
                              10 * kEps));

  // Test when x is zero.
  x = InitializeAutoDiff(Eigen::Vector3d::Zero(), x_grad);
  norm = DifferentiableNorm(x);
  EXPECT_NEAR(norm.value(), 0., 10 * kEps);
  EXPECT_TRUE(CompareMatrices(norm.derivatives(), Vector1d::Zero(), 10 * kEps));
}

}  // namespace
}  // namespace math
}  // namespace drake
