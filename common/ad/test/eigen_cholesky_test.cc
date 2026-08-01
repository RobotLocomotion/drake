#include <gtest/gtest.h>

#include "drake/common/ad/auto_diff.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/math/autodiff.h"
#include "drake/math/autodiff_gradient.h"

// Regression tests for drake#17037: Eigen's triangular solvers skip the
// divide-and-propagate step for any right-hand-side component that compares
// equal to zero. Without a derivative-aware is_identically_zero specialization
// (see eigen_specializations.h), a component whose value is exactly zero but
// whose derivatives are nonzero has its derivatives silently dropped by
// dynamic-size llt() and ldlt() solves. The systems below are constructed to
// hit exactly that case, and the expected gradients are the analytic inverse
// of the (constant) matrix: x = M⁻¹b implies ∂x/∂b = M⁻¹.

namespace drake {
namespace ad {
namespace {

using drake::math::ExtractGradient;
using drake::math::InitializeAutoDiff;
using Eigen::Matrix2d;
using Eigen::MatrixXd;
using Eigen::Vector2d;

constexpr bool kOldEigen = !EIGEN_VERSION_AT_LEAST(5, 0, 0);

constexpr double kTolerance = 1e-14;

// Returns b = [b0, b1] with ∂b/∂b = I₂, so that ∂x/∂b = M⁻¹ exactly.
VectorX<AutoDiff> MakeIndependentRhs(double b0, double b1) {
  return InitializeAutoDiff(Vector2d{b0, b1});
}

// Solving with llt() exercises the non-unit-diagonal triangular solve, where
// the zero-skip would omit the division of the derivatives by the Cholesky
// diagonal. x[0] has value zero with nonzero derivatives.
GTEST_TEST(EigenCholeskyTest, LltDynamicSolve) {
  MatrixX<AutoDiff> M = MatrixX<AutoDiff>::Zero(2, 2);
  M(0, 0) = 4.0;
  M(1, 1) = 9.0;
  const VectorX<AutoDiff> b = MakeIndependentRhs(0.0, 5.0);
  const VectorX<AutoDiff> x = M.llt().solve(b);

  const Vector2d value_expected(0.0, 5.0 / 9.0);
  MatrixXd gradient_expected =
      (Matrix2d() << 0.25, 0.0, 0.0, 1.0 / 9.0).finished();
  if (kOldEigen) {
    // Eigen 3.4.x lacks the is_identically_zero customization point, so expect
    // a *wrong* answer in that case. See drake#17037.
    gradient_expected(0, 0) = 1.0;
  }
  EXPECT_TRUE(
      CompareMatrices(ExtractGradient(x), gradient_expected, kTolerance));
  EXPECT_NEAR(x[0].value(), value_expected[0], kTolerance);
  EXPECT_NEAR(x[1].value(), value_expected[1], kTolerance);
}

// Solving with ldlt() exercises the unit-diagonal triangular solve, where the
// zero-skip would omit propagating the derivatives of the intermediate
// component y[0] = b[0] (value zero, nonzero derivatives) into y[1].
GTEST_TEST(EigenCholeskyTest, LdltDynamicSolve) {
  MatrixX<AutoDiff> M(2, 2);
  M(0, 0) = 4.0;
  M(0, 1) = 2.0;
  M(1, 0) = 2.0;
  M(1, 1) = 3.0;
  const VectorX<AutoDiff> b = MakeIndependentRhs(0.0, 1.0);
  const VectorX<AutoDiff> x = M.ldlt().solve(b);

  // M⁻¹ = (1/8) [3 -2; -2 4].
  MatrixXd gradient_expected =
      (Matrix2d() << 3.0 / 8.0, -2.0 / 8.0, -2.0 / 8.0, 4.0 / 8.0).finished();
  if (kOldEigen) {
    // Eigen 3.4.x lacks the is_identically_zero customization point, so expect
    // a *wrong* answer in that case. See drake#17037.
    gradient_expected(0, 0) = 0.25;
    gradient_expected(1, 0) = 0;
  }
  EXPECT_TRUE(
      CompareMatrices(ExtractGradient(x), gradient_expected, kTolerance));
  EXPECT_NEAR(x[0].value(), -0.25, kTolerance);
  EXPECT_NEAR(x[1].value(), 0.5, kTolerance);
}

// Fixed-size solves use Eigen's unrolled triangular solver, which has no
// zero-skip; this control case documents that boundary (and must keep passing
// on all supported Eigen versions).
GTEST_TEST(EigenCholeskyTest, LltFixedSizeSolve) {
  Matrix2<AutoDiff> M;
  M.setZero();
  M(0, 0) = 4.0;
  M(1, 1) = 9.0;
  Vector2<AutoDiff> b;
  b[0] = AutoDiff{0.0, Vector2d::Unit(0)};
  b[1] = AutoDiff{5.0, Vector2d::Unit(1)};
  const Vector2<AutoDiff> x = M.llt().solve(b);

  VectorX<AutoDiff> x_dynamic(2);
  x_dynamic << x[0], x[1];
  const MatrixXd gradient_expected =
      (Matrix2d() << 0.25, 0.0, 0.0, 1.0 / 9.0).finished();
  EXPECT_TRUE(CompareMatrices(ExtractGradient(x_dynamic), gradient_expected,
                              kTolerance));
}

}  // namespace
}  // namespace ad
}  // namespace drake
