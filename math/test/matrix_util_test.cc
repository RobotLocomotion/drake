#include "drake/math/matrix_util.h"

#include <gtest/gtest.h>

#include "drake/common/symbolic.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"

namespace drake {
namespace math {
namespace test {
GTEST_TEST(TestMatrixUtil, TestIsSymmetric) {
  auto A = Eigen::Matrix3d::Zero();
  EXPECT_TRUE(IsSymmetric(A, 0.0));
  EXPECT_TRUE(IsSymmetric(A.topLeftCorner<2, 2>(), 0.0));
  EXPECT_TRUE(IsSymmetric(Eigen::Matrix4d::Identity(), 0.0));
  Eigen::Matrix3d B;
  B << 1, 2, 3, 2, 4, 5, 3, 5, 6;
  EXPECT_TRUE(IsSymmetric(B, std::numeric_limits<double>::epsilon()));
  EXPECT_FALSE(IsSymmetric(Eigen::Matrix<double, 2, 3>::Zero(), 0.1));
  Eigen::Matrix3d C = B;
  C(0, 2) = 1.0;
  C(2, 0) = -1.0;
  EXPECT_FALSE(IsSymmetric(C, 1E-10));

  // Tests a integer scalar type.
  EXPECT_TRUE(IsSymmetric(Eigen::Matrix2i::Identity(), 0));
  Eigen::Matrix2i D;
  D << 0, 0, 1, 0;
  EXPECT_FALSE(IsSymmetric(D, 0));

  // Tests a symbolic expression.
  Eigen::Matrix<symbolic::Expression, 2, 2> S;
  symbolic::Variable s1{"s1"};
  symbolic::Variable s2{"s2"};
  symbolic::Variable s3{"s3"};
  symbolic::Expression S00{s1};
  symbolic::Expression S01{s2};
  symbolic::Expression S11{s3};
  S << S00, S01, S01, S11;
  EXPECT_TRUE(IsSymmetric(S));
}

GTEST_TEST(TestMatrixUtil, TestToSymmetricMatrixFromLowerTriangularColumns) {
  // Tests a static size vector.
  Eigen::Vector3d x1(1, 2, 3);
  Eigen::Matrix2d X1;
  // clang-format off
  X1 << 1, 2,
        2, 3;
  // clang-format on
  auto X1_result = ToSymmetricMatrixFromLowerTriangularColumns(x1);
  EIGEN_STATIC_ASSERT_SAME_MATRIX_SIZE(decltype(X1_result), Eigen::Matrix2d);
  EXPECT_TRUE(CompareMatrices(X1_result, X1));

  // Tests a dynamic size vector.
  Eigen::VectorXd x2(6);
  x2 << 1, 2, 3, 4, 5, 6;
  Eigen::Matrix3d X2;
  // clang-format off
  X2 << 1, 2, 3,
        2, 4, 5,
        3, 5, 6;
  // clang-format on
  EXPECT_TRUE(
      CompareMatrices(ToSymmetricMatrixFromLowerTriangularColumns(x2), X2));
}

GTEST_TEST(TestMatrixUtil, IsPositiveDefiniteTest) {
  Eigen::Matrix3d A;
  A << 1, 2, 4, 2, 3, 5, 4, 5, 6;
  EXPECT_FALSE(IsPositiveDefinite(A, 1e-8));

  EXPECT_TRUE(IsPositiveDefinite(A * A.transpose(), 1e-8));

  // A is full rank and all its eigen values >= 0
  // However A is not symmetric, thus not PSD
  A << 3, 2, 1, 2, 3, 1, 1, 2, 3;
  EXPECT_FALSE(IsPositiveDefinite(A, 1e-8));
  // But if the tolerance for symmetry testing is absurd, then it passes:
  EXPECT_TRUE(IsPositiveDefinite(A, 1e-8, 5.));

  // Generate a PSD matrix.
  EXPECT_FALSE(IsPositiveDefinite(
      A * Eigen::Vector3d(1., 0., 1.).asDiagonal() * A.transpose(), 1e-8));
}

}  // namespace test
}  // namespace math
}  // namespace drake
