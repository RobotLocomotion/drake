#include <gtest/gtest.h>

#include "drake/common/ad/auto_diff.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"

namespace drake {
namespace ad {
namespace {

using Eigen::Vector3d;

GTEST_TEST(MatrixTest, MatrixProduct) {
  const AutoDiff x{0.4, Vector3d::LinSpaced(0.0, 2.0)};
  const AutoDiff y{0.3, Vector3d::LinSpaced(-1.0, 1.0)};

  // clang-format off
  MatrixX<AutoDiff> A(2, 3);
  A << 0, x, y,
       y, x, 0;
  MatrixX<AutoDiff> B(3, 1);
  B << x,
       y,
       0;
  // clang-format on

  // Check the ScalarBinaryOpTraits by multiplying mixed-type scalars.
  EXPECT_TRUE((std::is_same_v<decltype(A * 2.0)::Scalar, AutoDiff>));
  EXPECT_TRUE((std::is_same_v<decltype(2.0 * B)::Scalar, AutoDiff>));
  MatrixX<AutoDiff> A2 = A * 2.0;
  MatrixX<AutoDiff> B2 = 2.0 * B;

  // Spot check some matrix multiplication. Between the partials_test and the
  // standard_operations test this probably will never catch any novel bugs, but
  // it's easy enough to check that it seems worthwhile.
  MatrixX<AutoDiff> C = A2 * B2;
  // We expect:
  //  [[ 4xy ]
  //   [ 8xy ]]
  // We don't need a tight tolerance, these spot checks are all-or-nothing.
  constexpr double tol = 1e-10;
  ASSERT_EQ(C.rows(), 2);
  ASSERT_EQ(C.cols(), 1);
  EXPECT_NEAR(C(0, 0).value(), 4 * 0.4 * 0.3, tol);
  EXPECT_NEAR(C(1, 0).value(), 8 * 0.4 * 0.3, tol);
  const Vector3d xy_grad(-0.4, 0.3, 1.0);  // (xy)' = yx' + xy'
  EXPECT_TRUE(CompareMatrices(C(0, 0).derivatives(), 4 * xy_grad, tol));
  EXPECT_TRUE(CompareMatrices(C(1, 0).derivatives(), 8 * xy_grad, tol));
}

}  // namespace
}  // namespace ad
}  // namespace drake
