#include "drake/math/quadratic_form.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"

namespace drake {
namespace math {
namespace {
void CheckDecomposePositiveQuadraticForm(const Eigen::Ref<const Eigen::MatrixXd>& Q, const Eigen::Ref<const Eigen::VectorXd>& b, double c) {
  Eigen::MatrixXd R;
  Eigen::VectorXd d;
  std::tie(R, d) = DecomposePositiveQuadraticForm(Q, b, c);
  EXPECT_TRUE(CompareMatrices(R.transpose() * R, Q, 1E-10, MatrixCompareType::absolute));
  EXPECT_TRUE(CompareMatrices(R.transpose() * d, b / 2, 1E-10, MatrixCompareType::absolute));
  EXPECT_NEAR(d.squaredNorm(), c, 1E-10);
}

GTEST_TEST(TestDecomposePositiveQuadraticForm, Test0) {
  // Decomposes a positive quadratic form without linear terms.
  // The quadratic form is 4x² + 4y² + 9.
  // This quadratic form is the same as zᵀ*z, where z is the vector
  // [2x]
  // [2y]
  // [ 3]
  Eigen::Matrix2d Q = 4 * Eigen::Matrix2d::Identity();
  Eigen::Vector2d b = Eigen::Vector2d::Zero();
  double c = 9;
  CheckDecomposePositiveQuadraticForm(Q, b, c);
}

GTEST_TEST(TestDecomposePositiveQuadraticForm, Test1) {
  // Decomposes a positive quadratic form with linear terms.
  //   x² + 4xy + 4y² + 2x + 4y + 2
  // = (x + 2y + 1)² + 1
  Eigen::Matrix2d Q;
  Q << 1, 2, 2, 4;
  Eigen::Vector2d b(2, 4);
  double c = 2;
  CheckDecomposePositiveQuadraticForm(Q, b, c);
}

GTEST_TEST(TestDecomposePositiveQuadraticForm, Test2) {
  // Decomposes a positive quadratic form with both linear and cross
  // terms.
  // x² + 2xy + 4y² + 4y +4
  Eigen::Matrix2d Q;
  Q << 1, 1, 1, 4;
  Eigen::Vector2d b(0, 4);
  double c = 2;
  CheckDecomposePositiveQuadraticForm(Q, b, c);
}

GTEST_TEST(TestDecomposePositiveQuadraticForm, Test3) {
  // Decomposes a positive form with no constant term.
  // x² + 4xy + 4y²
  Eigen::Matrix2d Q;
  Q << 1, 2, 2, 4;
  Eigen::Vector2d b(0, 0);
  double c = 0;
  CheckDecomposePositiveQuadraticForm(Q, b, c);
}
}  // namespace
}  // namespace math
}  // namespace drake
