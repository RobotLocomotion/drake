#include "drake/math/discrete_algebraic_riccati_equation.h"

#include <gtest/gtest.h>

#include "drake/common/eigen_matrix_compare.h"
#include "drake/math/autodiff.h"

using Eigen::MatrixXd;

namespace drake {
namespace math {
namespace {
void SolveDAREandVerify(const Eigen::Ref<const MatrixXd>& A,
                        const Eigen::Ref<const MatrixXd>& B,
                        const Eigen::Ref<const MatrixXd>& Q,
                        const Eigen::Ref<const MatrixXd>& R) {
  MatrixXd X = DiscreteAlgebraicRiccatiEquation(A, B, Q, R);
  // Check that X is positive semi-definite.
  EXPECT_TRUE(
      CompareMatrices(X, X.transpose(), 1E-10, MatrixCompareType::absolute));
  int n = X.rows();
  Eigen::SelfAdjointEigenSolver<MatrixXd> es(X);
  for (int i = 0; i < n; i++) {
    EXPECT_GE(es.eigenvalues()[i], 0);
  }
  // Check that X is the solution to the discrete time ARE.
  MatrixXd Y = A.transpose() * X * A - X -
               A.transpose() * X * B * (B.transpose() * X * B + R).inverse() *
                   B.transpose() * X * A +
               Q;
  EXPECT_TRUE(CompareMatrices(Y, MatrixXd::Zero(n, n), 1E-10,
                              MatrixCompareType::absolute));
}

GTEST_TEST(DARE, SolveDAREandVerify) {
  // Test 1: non-invertible A
  // Example 2 of "On the Numerical Solution of the Discrete-Time Algebraic
  // Riccati Equation"
  int n1 = 4, m1 = 1;
  MatrixXd A1(n1, n1), B1(n1, m1), Q1(n1, n1), R1(m1, m1);
  A1 << 0.5, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0;
  B1 << 0, 0, 0, 1;
  Q1 << 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
  R1 << 0.25;
  SolveDAREandVerify(A1, B1, Q1, R1);
  // Test 2: invertible A
  int n2 = 2, m2 = 1;
  MatrixXd A2(n2, n2), B2(n2, m2), Q2(n2, n2), R2(m2, m2);
  A2 << 1, 1, 0, 1;
  B2 << 0, 1;
  Q2 << 1, 0, 0, 0;
  R2 << 0.3;
  SolveDAREandVerify(A2, B2, Q2, R2);
}
}  // namespace
}  // namespace math
}  // namespace drake
