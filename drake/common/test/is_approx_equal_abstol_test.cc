#include "drake/common/is_approx_equal_abstol.h"

#include <Eigen/Dense>
#include <gtest/gtest.h>

using Eigen::MatrixXd;
using Eigen::VectorXd;

namespace drake {
namespace {

GTEST_TEST(IsApproxEqualMatrixTest, BasicTest) {
  const VectorXd ones1 = VectorXd::Ones(1);
  const VectorXd ones2 = VectorXd::Ones(2);
  const MatrixXd id22 = MatrixXd::Identity(2, 2);
  const MatrixXd id33 = MatrixXd::Identity(3, 3);

  const double lit_nudge = 1e-12;
  const double big_nudge = 1e-4;
  const VectorXd ones1_lit = ones1 + VectorXd::Constant(1, lit_nudge);
  const VectorXd ones1_big = ones1 + VectorXd::Constant(1, big_nudge);
  const VectorXd ones2_lit = ones2 + VectorXd::Constant(2, lit_nudge);
  const VectorXd ones2_big = ones2 + VectorXd::Constant(2, big_nudge);
  const MatrixXd id22_lit = id22 + MatrixXd::Constant(2, 2, lit_nudge);
  const MatrixXd id22_big = id22 + MatrixXd::Constant(2, 2, big_nudge);
  const MatrixXd id33_lit = id33 + MatrixXd::Constant(3, 3, lit_nudge);
  const MatrixXd id33_big = id33 + MatrixXd::Constant(3, 3, big_nudge);

  // Compare same-size matrices, within tolerances.
  const double tolerance = 1e-8;
  EXPECT_TRUE(is_approx_equal_abstol(ones1, ones1, tolerance));
  EXPECT_TRUE(is_approx_equal_abstol(ones1, ones1_lit, tolerance));
  EXPECT_FALSE(is_approx_equal_abstol(ones1, ones1_big, tolerance));
  EXPECT_TRUE(is_approx_equal_abstol(ones2, ones2, tolerance));
  EXPECT_TRUE(is_approx_equal_abstol(ones2, ones2_lit, tolerance));
  EXPECT_FALSE(is_approx_equal_abstol(ones2, ones2_big, tolerance));
  EXPECT_TRUE(is_approx_equal_abstol(id22, id22, tolerance));
  EXPECT_TRUE(is_approx_equal_abstol(id22, id22_lit, tolerance));
  EXPECT_FALSE(is_approx_equal_abstol(id22, id22_big, tolerance));
  EXPECT_TRUE(is_approx_equal_abstol(id33, id33, tolerance));
  EXPECT_TRUE(is_approx_equal_abstol(id33, id33_lit, tolerance));
  EXPECT_FALSE(is_approx_equal_abstol(id33, id33_big, tolerance));

  // Compare different-size matrices.
  EXPECT_FALSE(is_approx_equal_abstol(ones1, ones2, tolerance));
  EXPECT_FALSE(is_approx_equal_abstol(id22, id33, tolerance));

  // Special values do not compare equal.
  const double inf = std::numeric_limits<double>::infinity();
  const VectorXd inf2 = VectorXd::Constant(2, inf);
  EXPECT_FALSE(is_approx_equal_abstol(inf2, inf2, tolerance));
  EXPECT_FALSE(is_approx_equal_abstol(inf2, ones2, tolerance));
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const VectorXd nan2 = VectorXd::Constant(2, nan);
  EXPECT_FALSE(is_approx_equal_abstol(nan2, nan2, tolerance));
  EXPECT_FALSE(is_approx_equal_abstol(nan2, inf2, tolerance));
  EXPECT_FALSE(is_approx_equal_abstol(nan2, ones2, tolerance));
}

GTEST_TEST(IsApproxEqualAbstolPermutationTest, PermutationTest) {
  MatrixXd test(2, 3);
  // clang-format off
  test << 1, 2, 3,
          4, 5, 6;
  // clang-format on

  const double tol = 1e-8;
  EXPECT_TRUE(IsApproxEqualAbsTolWithPermutedColumns(test, test, tol));
  EXPECT_FALSE(
      IsApproxEqualAbsTolWithPermutedColumns(test, test.leftCols<2>(), tol));
  EXPECT_FALSE(
      IsApproxEqualAbsTolWithPermutedColumns(test.leftCols<2>(), test, tol));

  MatrixXd test2(2, 3);

  // Switch cols 2 and 3.
  // clang-format off
  test2 << 1, 3, 2,
           4, 6, 5;
  // clang-format on
  EXPECT_TRUE(IsApproxEqualAbsTolWithPermutedColumns(test, test2, tol));

  // All columns in test2 are in test1, but one is repeated.
  // clang-format off
  test2 << 1, 1, 2,
           4, 4, 5;
  // clang-format on
  EXPECT_FALSE(IsApproxEqualAbsTolWithPermutedColumns(test, test2, tol));
  EXPECT_FALSE(IsApproxEqualAbsTolWithPermutedColumns(test2, test, tol));

  // Matching but with one duplicated columns.
  test2.resize(2, 4);
  // clang-format off
  test2 << 1, 1, 2, 3,
           4, 4, 5, 6;
  // clang-format on
  EXPECT_FALSE(IsApproxEqualAbsTolWithPermutedColumns(test, test2, tol));
  EXPECT_FALSE(IsApproxEqualAbsTolWithPermutedColumns(test2, test, tol));
}

}  // namespace
}  // namespace drake
