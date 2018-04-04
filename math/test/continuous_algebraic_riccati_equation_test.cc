#include "drake/math/continuous_algebraic_riccati_equation.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"

using Eigen::MatrixXd;

namespace drake {
namespace math {
namespace {

void SolveCAREandVerify(const Eigen::Ref<const MatrixXd>& A,
                        const Eigen::Ref<const MatrixXd>& B,
                        const Eigen::Ref<const MatrixXd>& Q,
                        const Eigen::Ref<const MatrixXd>& R) {
  MatrixXd X = ContinuousAlgebraicRiccatiEquation(
      A, B, Q, R);

  EXPECT_TRUE(
      CompareMatrices(X, X.transpose(), 1E-10, MatrixCompareType::absolute));
  int n = X.rows();

  // Checks X is positive definite.
  Eigen::LLT<Eigen::MatrixXd> llt(X);
  EXPECT_EQ(llt.info(), Eigen::Success);

  // Tests the solution:
  // A^T * X + X * A - X * B * R^-1 * B^T * X + Q = 0
  MatrixXd Y = (A.transpose() * X) + (X * A) -
               (X * B * R.inverse() * B.transpose() * X) + Q;

  EXPECT_TRUE(CompareMatrices(Y, MatrixXd::Zero(n, n), 1E-10,
                              MatrixCompareType::absolute));

  // Checks the closed loop system's stability.
  MatrixXd K = R.inverse() * B.transpose() * X;
  Eigen::EigenSolver<MatrixXd> es(A - B * K);
  for (int i = 0; i < n; i++) {
    EXPECT_LT(es.eigenvalues()[i].real(), 0);
  }
}

GTEST_TEST(CARE, TestCare) {
  MatrixXd A1(2, 2), B1(2, 1), Q(2, 2), R1(1, 1);
  A1 << -3, 2, 1, 1;
  B1 << 0, 1;
  Q << 3, 0, 0, 3;
  R1 << 3;
  SolveCAREandVerify(A1, B1, Q, R1);
}

// Test the stabilization of the linearized pendulum (at the top).
GTEST_TEST(CARE, TestCare2) {
  MatrixXd A1(2, 2), B1(2, 1), Q(2, 2), R1(1, 1);
  A1 << 0, 1, 10, 0;
  B1 << 0, 1;
  Q << 1, 0, 0, 1;
  R1 << 1;
  SolveCAREandVerify(A1, B1, Q, R1);
}

}  // namespace
}  // namespace math
}  // namespace drake
