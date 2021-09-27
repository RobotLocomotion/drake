#include "drake/math/normalize_vector.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/math/autodiff.h"
#include "drake/math/autodiff_gradient.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;

namespace drake {
namespace math {
namespace {

// Tests the value and the gradient of NormalizeVector function.
template<int nx>
void NormalizeVectorTestFun(const Eigen::Matrix<double, nx, 1>& x) {
  Eigen::Matrix<double, nx, 1> x_normalized;
  typename Gradient<Eigen::Matrix<double, nx, 1>, nx, 1>::type dx_normalized;
  typename Gradient<Eigen::Matrix<double, nx, 1>, nx, 2>::type ddx_normalized;
  NormalizeVector(x, x_normalized, &dx_normalized, &ddx_normalized);

  // Now computes the gradient from autodiff.
  auto x_autodiff = InitializeAutoDiff(x);
  auto x_norm = x_autodiff.norm();
  auto x_normalized_autodiff = x_autodiff / x_norm;

  EXPECT_TRUE(CompareMatrices(
      x_normalized, ExtractValue(x_normalized_autodiff),
      1E-10, MatrixCompareType::absolute));
  EXPECT_TRUE(CompareMatrices(
      dx_normalized, ExtractGradient(x_normalized_autodiff),
      1E-10, MatrixCompareType::absolute));
}

GTEST_TEST(NormalizeVectorTest, NormalizeVector) {
  NormalizeVectorTestFun<2>(Eigen::Vector2d(1.0, 0.0));
  NormalizeVectorTestFun<3>(Eigen::Vector3d(1.0, 2.0, 3.0));
}
}  // namespace
}  // namespace math
}  // namespace drake
