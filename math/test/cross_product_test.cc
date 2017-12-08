#include "drake/math/cross_product.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"

using Eigen::Matrix3d;
using Eigen::Vector3d;

namespace drake {
namespace math {
namespace {
void SkewSymMatTestFun(const Vector3d& x) {
  // Manually computes the skew symmetric matrix.
  Matrix3d x_skew_mat_expected;
  x_skew_mat_expected << 0, -x(2), x(1), x(2), 0, -x(0), -x(1), x(0), 0;
  auto x_skew_mat = VectorToSkewSymmetric(x);
  EXPECT_TRUE(CompareMatrices(x_skew_mat_expected, x_skew_mat, 1E-10,
                              MatrixCompareType::absolute));
  // Checks the skew-symmetric property A' = -A.
  EXPECT_TRUE(CompareMatrices(x_skew_mat, -x_skew_mat.transpose(), 1E-10,
                              MatrixCompareType::absolute));
}

GTEST_TEST(CrossProductTest, SkewSymMatTest) {
  SkewSymMatTestFun(Vector3d(0.0, 0.0, 0.0));
  SkewSymMatTestFun(Vector3d(1.0, 0.0, 0.0));
  SkewSymMatTestFun(Vector3d(0.0, 1.0, 0.0));
  SkewSymMatTestFun(Vector3d(0.0, 0.0, 1.0));
  SkewSymMatTestFun(Vector3d(1.0, 2.0, 3.0));
}
}  // namespace
}  // namespace math
}  // namespace drake
