#include "drake/math/discrete_algebraic_riccati_equation.h"

#include <exception>

#include <Eigen/LU>  // for inverse()
#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"

namespace drake {
namespace math {
namespace {

void SolveDAREandVerify(const Eigen::Ref<const Eigen::MatrixXd>& A,
                        const Eigen::Ref<const Eigen::MatrixXd>& B,
                        const Eigen::Ref<const Eigen::MatrixXd>& Q,
                        const Eigen::Ref<const Eigen::MatrixXd>& R) {
  int states = A.rows();

  Eigen::MatrixXd X = DiscreteAlgebraicRiccatiEquation(A, B, Q, R);

  // Check that X is symmetric
  EXPECT_TRUE(
      CompareMatrices(X, X.transpose(), 1e-10, MatrixCompareType::absolute));

  // Check that X is positive semidefinite
  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> es{X, Eigen::EigenvaluesOnly};
  for (int i = 0; i < states; ++i) {
    EXPECT_GE(es.eigenvalues()[i], 0);
  }

  // Check that X is the solution to the discrete time ARE
  // clang-format off
  Eigen::MatrixXd Y =
      A.transpose() * X * A
      - X
      - (A.transpose() * X * B * (B.transpose() * X * B + R).inverse()
        * B.transpose() * X * A)
      + Q;
  // clang-format on
  EXPECT_TRUE(CompareMatrices(Y, Eigen::MatrixXd::Zero(states, states), 1e-10,
                              MatrixCompareType::absolute));
}

void SolveDAREandVerify(const Eigen::Ref<const Eigen::MatrixXd>& A,
                        const Eigen::Ref<const Eigen::MatrixXd>& B,
                        const Eigen::Ref<const Eigen::MatrixXd>& Q,
                        const Eigen::Ref<const Eigen::MatrixXd>& R,
                        const Eigen::Ref<const Eigen::MatrixXd>& N) {
  int states = A.rows();

  Eigen::MatrixXd X = DiscreteAlgebraicRiccatiEquation(A, B, Q, R, N);

  // Check that X is symmetric
  EXPECT_TRUE(
      CompareMatrices(X, X.transpose(), 1e-10, MatrixCompareType::absolute));

  // Check that X is positive semidefinite
  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> es{X, Eigen::EigenvaluesOnly};
  for (int i = 0; i < states; ++i) {
    EXPECT_GE(es.eigenvalues()[i], 0);
  }

  // Check that X is the solution to the discrete time ARE
  // clang-format off
  Eigen::MatrixXd Y =
      A.transpose() * X * A
      - X
      - ((A.transpose() * X * B + N) * (B.transpose() * X * B + R).inverse()
        * (B.transpose() * X * A + N.transpose()))
      + Q;
  // clang-format on
  EXPECT_TRUE(CompareMatrices(Y, Eigen::MatrixXd::Zero(states, states), 1e-10,
                              MatrixCompareType::absolute));
}

GTEST_TEST(DARE, NonInvertibleA) {
  // Test 1: non-invertible A
  // Example 2 of "On the Numerical Solution of the Discrete-Time Algebraic
  // Riccati Equation"
  Eigen::MatrixXd A{4, 4};
  A << 0.5, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0;
  Eigen::MatrixXd B{4, 1};
  B << 0, 0, 0, 1;
  Eigen::MatrixXd Q{4, 4};
  Q << 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
  Eigen::MatrixXd R{1, 1};
  R << 0.25;
  SolveDAREandVerify(A, B, Q, R);

  Eigen::MatrixXd Aref{4, 4};
  Aref << 0.25, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0;
  SolveDAREandVerify(A, B, (A - Aref).transpose() * Q * (A - Aref),
                     B.transpose() * Q * B + R, (A - Aref).transpose() * Q * B);
}

GTEST_TEST(DARE, InvertibleA) {
  // Test 2: invertible A
  Eigen::MatrixXd A{2, 2};
  A << 1, 1, 0, 1;
  Eigen::MatrixXd B{2, 1};
  B << 0, 1;
  Eigen::MatrixXd Q{2, 2};
  Q << 1, 0, 0, 0;
  Eigen::MatrixXd R{1, 1};
  R << 0.3;
  SolveDAREandVerify(A, B, Q, R);

  Eigen::MatrixXd Aref{2, 2};
  Aref << 0.5, 1, 0, 1;
  SolveDAREandVerify(A, B, (A - Aref).transpose() * Q * (A - Aref),
                     B.transpose() * Q * B + R, (A - Aref).transpose() * Q * B);
}

GTEST_TEST(DARE, FirstGeneralizedEigenvalueOfSTIsStable) {
  // Test 3: the first generalized eigenvalue of (S, T) is stable
  Eigen::MatrixXd A{2, 2};
  A << 0, 1, 0, 0;
  Eigen::MatrixXd B{2, 1};
  B << 0, 1;
  Eigen::MatrixXd Q{2, 2};
  Q << 1, 0, 0, 1;
  Eigen::MatrixXd R{1, 1};
  R << 1;
  SolveDAREandVerify(A, B, Q, R);

  Eigen::MatrixXd Aref{2, 2};
  Aref << 0, 0.5, 0, 0;
  SolveDAREandVerify(A, B, (A - Aref).transpose() * Q * (A - Aref),
                     B.transpose() * Q * B + R, (A - Aref).transpose() * Q * B);
}

GTEST_TEST(DARE, IdentitySystem) {
  // Test 4: A = B = Q = R = I₂ₓ₂
  const Eigen::MatrixXd A{Eigen::Matrix2d::Identity()};
  const Eigen::MatrixXd B{Eigen::Matrix2d::Identity()};
  const Eigen::MatrixXd Q{Eigen::Matrix2d::Identity()};
  const Eigen::MatrixXd R{Eigen::Matrix2d::Identity()};
  SolveDAREandVerify(A, B, Q, R);

  const Eigen::MatrixXd N{Eigen::Matrix2d::Identity()};
  SolveDAREandVerify(A, B, Q, R, N);
}

GTEST_TEST(DARE, MoreInputsThanStates) {
  // Test 5: More inputs than states
  const Eigen::Matrix2d A{Eigen::Matrix2d::Identity()};
  Eigen::MatrixXd B{2, 3};
  B << 1, 0, 0, 0, 0.5, 0.3;
  const Eigen::Matrix2d Q{Eigen::Matrix2d::Identity()};
  const Eigen::Matrix3d R{Eigen::Matrix3d::Identity()};
  SolveDAREandVerify(A, B, Q, R);

  Eigen::MatrixXd N{2, 3};
  N << 1, 0, 0, 0, 1, 0;
  SolveDAREandVerify(A, B, Q, R, N);
}

GTEST_TEST(DARE, QNotSymmetricPositiveSemidefinite_ABQR) {
  const Eigen::Matrix2d A{Eigen::Matrix2d::Identity()};
  const Eigen::Matrix2d B{Eigen::Matrix2d::Identity()};
  const Eigen::Matrix2d Q{-Eigen::Matrix2d::Identity()};
  const Eigen::Matrix2d R{Eigen::Matrix2d::Identity()};

  EXPECT_THROW(SolveDAREandVerify(A, B, Q, R), std::exception);
}

GTEST_TEST(DARE, QNotSymmetricPositiveSemidefinite_ABQRN) {
  const Eigen::Matrix2d A{Eigen::Matrix2d::Identity()};
  const Eigen::Matrix2d B{Eigen::Matrix2d::Identity()};
  const Eigen::Matrix2d Q{Eigen::Matrix2d::Identity()};
  const Eigen::Matrix2d R{Eigen::Matrix2d::Identity()};
  const Eigen::Matrix2d N{2.0 * Eigen::Matrix2d::Identity()};

  EXPECT_THROW(SolveDAREandVerify(A, B, Q, R, N), std::exception);
}

GTEST_TEST(DARE, RNotSymmetricPositiveDefinite_ABQR) {
  const Eigen::Matrix2d A{Eigen::Matrix2d::Identity()};
  const Eigen::Matrix2d B{Eigen::Matrix2d::Identity()};
  const Eigen::Matrix2d Q{Eigen::Matrix2d::Identity()};

  const Eigen::Matrix2d R1{Eigen::Matrix2d::Zero()};
  EXPECT_THROW(SolveDAREandVerify(A, B, Q, R1), std::exception);

  const Eigen::Matrix2d R2{-Eigen::Matrix2d::Identity()};
  EXPECT_THROW(SolveDAREandVerify(A, B, Q, R2), std::exception);
}

GTEST_TEST(DARE, RNotSymmetricPositiveDefinite_ABQRN) {
  const Eigen::Matrix2d A{Eigen::Matrix2d::Identity()};
  const Eigen::Matrix2d B{Eigen::Matrix2d::Identity()};
  const Eigen::Matrix2d Q{Eigen::Matrix2d::Identity()};
  const Eigen::Matrix2d N{Eigen::Matrix2d::Identity()};

  const Eigen::Matrix2d R1{Eigen::Matrix2d::Zero()};
  EXPECT_THROW(SolveDAREandVerify(A, B, Q, R1, N), std::exception);

  const Eigen::Matrix2d R2{-Eigen::Matrix2d::Identity()};
  EXPECT_THROW(SolveDAREandVerify(A, B, Q, R2, N), std::exception);
}

GTEST_TEST(DARE, ABNotStabilizable_ABQR) {
  const Eigen::Matrix2d A{Eigen::Matrix2d::Identity()};
  const Eigen::Matrix2d B{Eigen::Matrix2d::Zero()};
  const Eigen::Matrix2d Q{Eigen::Matrix2d::Identity()};
  const Eigen::Matrix2d R{Eigen::Matrix2d::Identity()};

  EXPECT_THROW(SolveDAREandVerify(A, B, Q, R), std::exception);
}

GTEST_TEST(DARETest, ABNotStabilizable_ABQRN) {
  const Eigen::Matrix2d A{Eigen::Matrix2d::Identity()};
  const Eigen::Matrix2d B{Eigen::Matrix2d::Zero()};
  const Eigen::Matrix2d Q{Eigen::Matrix2d::Identity()};
  const Eigen::Matrix2d R{Eigen::Matrix2d::Identity()};
  const Eigen::Matrix2d N{Eigen::Matrix2d::Identity()};

  EXPECT_THROW(SolveDAREandVerify(A, B, Q, R, N), std::exception);
}

GTEST_TEST(DARETest, ACNotDetectable_ABQR) {
  const Eigen::Matrix2d A{Eigen::Matrix2d::Identity()};
  const Eigen::Matrix2d B{Eigen::Matrix2d::Identity()};
  const Eigen::Matrix2d Q{Eigen::Matrix2d::Zero()};
  const Eigen::Matrix2d R{Eigen::Matrix2d::Identity()};

  EXPECT_THROW(SolveDAREandVerify(A, B, Q, R), std::exception);
}

GTEST_TEST(DARETest, ACNotDetectable_ABQRN) {
  const Eigen::Matrix2d A{Eigen::Matrix2d::Identity()};
  const Eigen::Matrix2d B{Eigen::Matrix2d::Identity()};
  const Eigen::Matrix2d Q{Eigen::Matrix2d::Zero()};
  const Eigen::Matrix2d R{Eigen::Matrix2d::Identity()};
  const Eigen::Matrix2d N{Eigen::Matrix2d::Zero()};

  EXPECT_THROW(SolveDAREandVerify(A, B, Q, R, N), std::exception);
}

GTEST_TEST(DARETest, QDecomposition) {
  // Ensures the decomposition of Q into CᵀC is correct

  Eigen::Matrix2d A{2, 2};
  A << 1.0, 0.0, 0.0, 0.0;
  const Eigen::Matrix2d B{Eigen::Matrix2d::Identity()};
  const Eigen::Matrix2d R{Eigen::Matrix2d::Identity()};

  // (A, C₁) should be detectable pair
  Eigen::Matrix2d C_1{2, 2};
  C_1 << 0.0, 0.0, 1.0, 0.0;
  const Eigen::Matrix2d Q_1 = C_1.transpose() * C_1;
  EXPECT_NO_THROW(SolveDAREandVerify(A, B, Q_1, R));

  // (A, C₂) shouldn't be detectable pair
  const Eigen::Matrix2d C_2 = C_1.transpose();
  const Eigen::Matrix2d Q_2 = C_2.transpose() * C_2;
  EXPECT_THROW(SolveDAREandVerify(A, B, Q_2, R), std::exception);
}

}  // namespace
}  // namespace math
}  // namespace drake
