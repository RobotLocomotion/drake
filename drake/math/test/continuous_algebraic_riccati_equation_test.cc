#include "drake/math/continuous_algebraic_riccati_equation.h"
#include <gtest/gtest.h>
#include "drake/common/eigen_matrix_compare.h"

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
  Eigen::SelfAdjointEigenSolver<MatrixXd> es(X);
  for (int i = 0; i < n; i++) {
    EXPECT_GE(es.eigenvalues()[i], 0);
  }
  // Test the solution:
  // A^T * X + X * A - X * B * R^-1 * B^T * X + Q = 0
  MatrixXd Y = (A.transpose() * X) + (X * A) -
               (X * B * R.inverse() * B.transpose() * X) + Q;

  EXPECT_TRUE(CompareMatrices(Y, MatrixXd::Zero(n, n), 1E-10,
                              MatrixCompareType::absolute));
}

GTEST_TEST(CARE, TestCare) {
  // Example from mathworks. Continuous-time algebraic Riccati equation solution
  MatrixXd A1(2, 2), B1(2, 1), Q(2, 2), R1(1, 1);
  A1 << -3, 2, 1, 1;
  B1 << 0, 1;
  Q << 3, 0, 0, 3;
  R1 << 3;
  SolveCAREandVerify(A1, B1, Q, R1);
}

}  // namespace
}  // namespace math
}  // namespace drake
