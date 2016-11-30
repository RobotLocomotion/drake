#include "drake/math/matrix_util.h"

#include <gtest/gtest.h>

namespace drake {
namespace math {
namespace test {
GTEST_TEST(TestMatrixUtil, TestIsSymmetric) {
  auto A = Eigen::Matrix3d::Zero();
  EXPECT_TRUE(IsSymmetric(A, 0.0));
  EXPECT_TRUE(IsSymmetric(A.topLeftCorner<2, 2>(), 0.0));
  EXPECT_TRUE(IsSymmetric(Eigen::Matrix4d::Identity(), 0.0));
  Eigen::Matrix4d B = Eigen::Matrix4d::Random();
  EXPECT_TRUE(IsSymmetric(B + B.transpose(), 2 * std::numeric_limits<double>::epsilon()));
  EXPECT_FALSE(IsSymmetric(Eigen::Matrix<double, 2, 3>::Random(), 0.1));
  Eigen::Matrix4d C = B + B.transpose();
  C(0, 2) = 1.0;
  C(2, 0) = -1.0;
  EXPECT_FALSE(IsSymmetric(C, 1E-10));
}
}
}
}