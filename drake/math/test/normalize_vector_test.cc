#include "drake/math/normalize_vector.h"

#include "gtest/gtest.h"

#include "drake/common/eigen_matrix_compare.h"
#include "drake/math/autodiff.h"
#include "drake/math/autodiff_gradient.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;

namespace drake {
namespace math {
namespace {

void NormalizeVectorTestFun(const Eigen::VectorXd& x) {
  VectorXd x_normalized(x.size());
  MatrixXd dx_normalized(x.size(), x.size());
  MatrixXd ddx_normalized(x.size() * x.size(), x.size());
  NormalizeVector(x, x_normalized, &dx_normalized, &ddx_normalized);

  // now computes the gradient from autodiff
  auto x_autodiff = initializeAutoDiff(x);
  auto x_norm = x_autodiff.norm();
  auto x_normalized_autodiff = x_autodiff / x_norm;

  EXPECT_TRUE(CompareMatrices(x_normalized,
                              autoDiffToValueMatrix(x_normalized_autodiff),
                              1E-10, MatrixCompareType::absolute));
  EXPECT_TRUE(CompareMatrices(dx_normalized,
                              autoDiffToGradientMatrix(x_normalized_autodiff),
                              1E-10, MatrixCompareType::absolute));
}

GTEST_TEST(NormalizeVectorTest, NormalizeVector) {
  NormalizeVectorTestFun(Eigen::Vector2d(1.0, 0.0));
  NormalizeVectorTestFun(Eigen::Vector3d(1.0, 2.0, 3.0));
}
}  // namespace
}  // namespace math
}  // namespace drake
