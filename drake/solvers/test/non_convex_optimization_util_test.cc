#include "drake/solvers/non_convex_optimization_util.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"

namespace drake {
namespace solvers {
namespace {
GTEST_TEST(DecomposeNonConvexQuadraticForm, Test0) {
  // Decomposes a PSD matrix Q. This should yield Q1 = Q and Q2 = 0.
  Eigen::Matrix3d Q1, Q2;
  Eigen::Matrix3d Q = Eigen::Matrix3d::Identity();
  std::tie(Q1, Q2) = DecomposeNonConvexQuadraticForm(Q);
  EXPECT_TRUE(CompareMatrices(Q1, Q, 1E-5, MatrixCompareType::absolute));
  EXPECT_TRUE(CompareMatrices(Q2, Eigen::Matrix3d::Zero(), 1E-5,
                              MatrixCompareType::absolute));
}

GTEST_TEST(DecomposeNonConvexQuadraticForm, Test1) {
  // Decomposes a negative definite matrix Q. This should yield Q1 = 0, and Q2 =
  // -Q.
  Eigen::Matrix3d Q1, Q2;
  Eigen::Matrix3d Q = -Eigen::Matrix3d::Identity();
  std::tie(Q1, Q2) = DecomposeNonConvexQuadraticForm(Q);
  EXPECT_TRUE(CompareMatrices(Q1, Eigen::Matrix3d::Zero(), 1E-5,
                              MatrixCompareType::absolute));
  EXPECT_TRUE(CompareMatrices(Q2, -Q, 1E-5, MatrixCompareType::absolute));
}

GTEST_TEST(DecomposeNonConvexQuadraticForm, Test2) {
  // Decomposes an indefinite matrix Q, [0, 1; 1 0].
  // This should yield Q1 = [1 1; 1 1]/2, Q2 = [1 -1; -1 1]/2.
  Eigen::Matrix2d Q1, Q2;
  Eigen::Matrix2d Q;
  Q << 0, 1, 1, 0;
  std::tie(Q1, Q2) = DecomposeNonConvexQuadraticForm(Q);
  Eigen::Matrix2d Q1_expected, Q2_expected;
  Q1_expected << 1.0 / 2, 1.0 / 2, 1.0 / 2, 1.0 / 2;
  Q2_expected << 1.0 / 2, -1.0 / 2, -1.0 / 2, 1.0 / 2;
  EXPECT_TRUE(
      CompareMatrices(Q1, Q1_expected, 1E-5, MatrixCompareType::absolute));
  EXPECT_TRUE(
      CompareMatrices(Q2, Q2_expected, 1E-5, MatrixCompareType::absolute));

  // Decomposes another indefinite matrix Q, [0, 2; 0, 0], this matrix has the
  // same quadratic form as [0, 1; 1 0], so it should give the same Q1 and Q2.
  Q << 0, 2, 0, 0;
  std::tie(Q1, Q2) = DecomposeNonConvexQuadraticForm(Q);
  EXPECT_TRUE(
      CompareMatrices(Q1, Q1_expected, 1E-5, MatrixCompareType::absolute));
  EXPECT_TRUE(
      CompareMatrices(Q2, Q2_expected, 1E-5, MatrixCompareType::absolute));
}

GTEST_TEST(DecomposeNonConvexQuadraticForm, Test3) {
  // Decomposes an indefinite matrix Q = [1 3; 1, 1].
  // This should yield Q1 = [1.5 1.5; 1.5 1.5], Q2 = [0.5 -0.5; -0.5 0.5]
  Eigen::Matrix2d Q1, Q2;
  Eigen::Matrix2d Q;
  Q << 1, 3, 1, 1;
  std::tie(Q1, Q2) = DecomposeNonConvexQuadraticForm(Q);
  Eigen::Matrix2d Q1_expected, Q2_expected;
  Q1_expected << 1.5, 1.5, 1.5, 1.5;
  Q2_expected << 0.5, -0.5, -0.5, 0.5;
  EXPECT_TRUE(
      CompareMatrices(Q1, Q1_expected, 1E-5, MatrixCompareType::absolute));
  EXPECT_TRUE(
      CompareMatrices(Q2, Q2_expected, 1E-5, MatrixCompareType::absolute));
}
}  // namespace
}  // namespace solvers
}  // namespace drake
