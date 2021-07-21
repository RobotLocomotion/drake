#include "drake/multibody/fixed_fem/dev/schur_complement.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"

namespace drake {
namespace multibody {
namespace fem {
namespace internal {
namespace {

using Eigen::Matrix2d;
using Eigen::MatrixXd;
using Eigen::Vector2d;
using Eigen::VectorXd;
/* Verify that SchurComplement provides the same answer as hand-calculated
solution on a small problem. */
GTEST_TEST(SchurComplementTest, AnalyticTest) {
  Matrix2d A, B, D;
  // clang-format off
  A << 3, 0,
       0, 2;
  B << 1, 1,
      -1, 0;
  D << 4, 1,
       1, 4;
  // clang-format on
  /* The matrix M is given by
     3 0  | 1  1
     0 2  | -1 0
     -----------
     1 -1 | 4  1
     1 0  | 1  4
  It is clearly symmetric and diagonally dominant and thus positive definite.
  The Schur complement for the bottom right block is given by
  A - BD⁻¹Bᵀ, which is equal to [13/5, 1/5; 1/5, 26/15] after simple
  calculation. */
  const SchurComplement<double> s(A.sparseView(), B.transpose().sparseView(),
                                  D.sparseView());
  Matrix2d expected_D_complement;
  // clang-format off
  expected_D_complement << 13./5., 1./5.,
                           1./5., 26./15.;
  // clang-format on
  EXPECT_TRUE(CompareMatrices(s.get_D_complement(), expected_D_complement,
                              std::numeric_limits<double>::epsilon()));

  /* Verify that y = -D⁻¹Bᵀx. */
  const Vector2d x(1, 2);
  const Vector2d expected_y(1. / 3., -1. / 3.);
  EXPECT_TRUE(CompareMatrices(s.SolveForY(x), expected_y,
                              std::numeric_limits<double>::epsilon()));
}

/* Verify that when M = D, the Schur complement is a zero-sized matrix and the
 solution to the y variable is zero. */
GTEST_TEST(SchurComplementTest, MisD) {
  const MatrixXd A(0, 0);
  const MatrixXd B(0, 2);
  Matrix2d D;
  // clang-format off
  D << 4, 1,
       1, 4;
  // clang-format on
  const SchurComplement<double> s(A.sparseView(), B.transpose().sparseView(),
                                  D.sparseView());
  EXPECT_EQ(s.get_D_complement().rows(), 0);
  EXPECT_EQ(s.get_D_complement().cols(), 0);
  /* Verify that y = 0. */
  const VectorXd x(0);
  EXPECT_TRUE(CompareMatrices(s.SolveForY(x), Vector2d::Zero()));
}

/* Verify that when M = A, the Schur complement is equal to A. */
GTEST_TEST(SchurComplementTest, MisA) {
  Matrix2d A;
  // clang-format off
  A << 4, 1,
       1, 4;
  // clang-format on
  const MatrixXd B(2, 0);
  const MatrixXd D(0, 0);
  const SchurComplement<double> s(A.sparseView(), B.transpose().sparseView(),
                                  D.sparseView());
  EXPECT_TRUE(CompareMatrices(s.get_D_complement(), A));
  /* Verify that the y variable is empty. */
  const VectorXd expected_y(0);
  const Vector2d x(1, 2);
  EXPECT_TRUE(CompareMatrices(s.SolveForY(x), expected_y));
}

}  // namespace
}  // namespace internal
}  // namespace fem
}  // namespace multibody
}  // namespace drake
