#include "drake/math/discrete_algebraic_riccati_equation.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
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
  // clang-format off
  MatrixXd Y =
      A.transpose() * X * A
      - X
      - (A.transpose() * X * B * (B.transpose() * X * B + R).inverse()
        * B.transpose() * X * A)
      + Q;
  // clang-format on
  EXPECT_TRUE(CompareMatrices(Y, MatrixXd::Zero(n, n), 1E-10,
                              MatrixCompareType::absolute));
}

void SolveDAREandVerify(const Eigen::Ref<const MatrixXd>& A,
                        const Eigen::Ref<const MatrixXd>& B,
                        const Eigen::Ref<const MatrixXd>& Q,
                        const Eigen::Ref<const MatrixXd>& R,
                        const Eigen::Ref<const MatrixXd>& N) {
  MatrixXd X = DiscreteAlgebraicRiccatiEquation(A, B, Q, R, N);
  // Check that X is positive semi-definite.
  EXPECT_TRUE(
      CompareMatrices(X, X.transpose(), 1E-10, MatrixCompareType::absolute));
  int n = X.rows();
  Eigen::SelfAdjointEigenSolver<MatrixXd> es(X);
  for (int i = 0; i < n; i++) {
    EXPECT_GE(es.eigenvalues()[i], 0);
  }
  // Check that X is the solution to the discrete time ARE.
  // clang-format off
  MatrixXd Y =
      A.transpose() * X * A
      - X
      - ((A.transpose() * X * B + N) * (B.transpose() * X * B + R).inverse()
        * (B.transpose() * X * A + N.transpose()))
      + Q;
  // clang-format on
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

  MatrixXd Aref1(n1, n1);
  Aref1 << 0.25, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0;
  SolveDAREandVerify(A1, B1, (A1 - Aref1).transpose() * Q1 * (A1 - Aref1),
      B1.transpose() * Q1 * B1 + R1, (A1 - Aref1).transpose() * Q1 * B1);

  // Test 2: invertible A
  int n2 = 2, m2 = 1;
  MatrixXd A2(n2, n2), B2(n2, m2), Q2(n2, n2), R2(m2, m2);
  A2 << 1, 1, 0, 1;
  B2 << 0, 1;
  Q2 << 1, 0, 0, 0;
  R2 << 0.3;
  SolveDAREandVerify(A2, B2, Q2, R2);

  MatrixXd Aref2(n2, n2);
  Aref2 << 0.5, 1, 0, 1;
  SolveDAREandVerify(A2, B2, (A2 - Aref2).transpose() * Q2 * (A2 - Aref2),
      B2.transpose() * Q2 * B2 + R2, (A2 - Aref2).transpose() * Q2 * B2);

  // Test 3: the first generalized eigenvalue of (S,T) is stable
  int n3 = 2, m3 = 1;
  MatrixXd A3(n3, n3), B3(n3, m3), Q3(n3, n3), R3(m3, m3);
  A3 << 0, 1, 0, 0;
  B3 << 0, 1;
  Q3 << 1, 0, 0, 1;
  R3 << 1;
  SolveDAREandVerify(A3, B3, Q3, R3);

  MatrixXd Aref3(n3, n3);
  Aref3 << 0, 0.5, 0, 0;
  SolveDAREandVerify(A3, B3, (A3 - Aref3).transpose() * Q3 * (A3 - Aref3),
      B3.transpose() * Q3 * B3 + R3, (A3 - Aref3).transpose() * Q3 * B3);

  // Test 4: A = B = Q = R = I_2 (2-by-2 identity matrix)
  const Eigen::MatrixXd A4{Eigen::Matrix2d::Identity()};
  const Eigen::MatrixXd B4{Eigen::Matrix2d::Identity()};
  const Eigen::MatrixXd Q4{Eigen::Matrix2d::Identity()};
  const Eigen::MatrixXd R4{Eigen::Matrix2d::Identity()};
  SolveDAREandVerify(A4, B4, Q4, R4);

  const Eigen::MatrixXd N4{Eigen::Matrix2d::Identity()};
  SolveDAREandVerify(A4, B4, Q4, R4, N4);
}
}  // namespace
}  // namespace math
}  // namespace drake
