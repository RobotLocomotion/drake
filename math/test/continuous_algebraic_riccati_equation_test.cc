#include "drake/math/continuous_algebraic_riccati_equation.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"

using Eigen::MatrixXd;

namespace drake {
namespace math {
namespace {

// @param is_X_pd X should always be positive semidefinite. Under some
// conditions on A, B, Q, the matrix X is positive definite. Check
// http://webpages.iust.ac.ir/b_moaveni/courses/paper_Riccati.pdf for the
// condition on whether the solution X is positive definite or positive
// semidefinite.
void SolveCAREandVerify(const Eigen::Ref<const MatrixXd>& A,
                        const Eigen::Ref<const MatrixXd>& B,
                        const Eigen::Ref<const MatrixXd>& Q,
                        const Eigen::Ref<const MatrixXd>& R, bool is_X_pd) {
  MatrixXd X = ContinuousAlgebraicRiccatiEquation(A, B, Q, R);

  EXPECT_TRUE(
      CompareMatrices(X, X.transpose(), 1E-10, MatrixCompareType::absolute));
  int n = X.rows();

  if (is_X_pd) {
    // Checks X is positive definite.
    Eigen::LLT<Eigen::MatrixXd> llt(X);
    EXPECT_EQ(llt.info(), Eigen::Success);
  } else {
    // Checks X is positive semidefinite.
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> es_X(X);
    EXPECT_TRUE((es_X.eigenvalues().array() >= -1E-10).all());
  }

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
  SolveCAREandVerify(A1, B1, Q, R1, true /* is_X_pd */);
}

// Test the stabilization of the linearized pendulum (at the top).
GTEST_TEST(CARE, TestCare2) {
  MatrixXd A1(2, 2), B1(2, 1), Q(2, 2), R1(1, 1);
  A1 << 0, 1, 10, 0;
  B1 << 0, 1;
  Q << 1, 0, 0, 1;
  R1 << 1;
  SolveCAREandVerify(A1, B1, Q, R1, true /* is_X_pd */);
}

// This test case is reported in
// https://github.com/RobotLocomotion/drake/issues/19191
GTEST_TEST(CARE, TestUndetectable) {
  // The pair (Q, A) is not detectable. Hence the continuous algebraic Riccati
  // equation will fail.
  Eigen::MatrixXd A = Eigen::MatrixXd::Zero(12, 12);
  A.topRightCorner<6, 6>() = Eigen::MatrixXd::Identity(6, 6);
  A(6, 0) = -12;
  A(6, 4) = 9;
  A(7, 5) = -9;

  Eigen::MatrixXd B = Eigen::MatrixXd::Zero(12, 4);
  // clang-format off
  B.bottomRows<4>() << 1, 1, 1, 1,
                       7, -7, 7, -7,
                       -59, 0, 59, 0,
                       0, 99, 0, -99;
  // clang-format on
  Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(12, 12);
  Q(0, 0) = 10;
  Q(1, 1) = 10;
  Q(2, 2) = 10;
  Eigen::MatrixXd R = Eigen::Matrix4d::Identity();
  DRAKE_EXPECT_THROWS_MESSAGE(ContinuousAlgebraicRiccatiEquation(A, B, Q, R),
                              "ContinuousAlgebraicRiccatiEquation fails.*");

  // Undetectable (Q, A)
  A.resize(2, 2);
  // clang-format off
  A << 1, 2,
       0, 2;
  // clang-format on
  Q.resize(2, 2);
  // clang-format off
  Q << 0, 0,
       0, 1;
  // clang-format on
  B.resize(2, 1);
  B << 1, 1;
  R.resize(1, 1);
  R << 1;
  DRAKE_EXPECT_THROWS_MESSAGE(ContinuousAlgebraicRiccatiEquation(A, B, Q, R),
                              "ContinuousAlgebraicRiccatiEquation fails. The "
                              "system is not detectable.");

  // Test an example in which (A, B) is stabilizable and (Q, A) is detectable
  // but (Q, A) is not observable. The example was created by Thomas Cohn in
  // https://github.com/RobotLocomotion/drake/pull/19222#issuecomment-1516939077
  A.resize(3, 3);
  // clang-format off
  A << 0, 1, 0,
       0, 0, 1,
       -1, -1, -1;
  // clang-format on
  B.resize(3, 1);
  B << 0, 0, 1;
  Q.resize(3, 3);
  // clang-format off
  Q << 1, 1, 0,
       1, 1, 0,
       0, 0, 0;
  // clang-format on
  R.resize(1, 1);
  R << 1;
  SolveCAREandVerify(A, B, Q, R, false /* is_X_pd */);
}

GTEST_TEST(Care, TestUnstabilizable) {
  // The pair (A, B) is not stabilizable.
  Eigen::Matrix2d A;
  // clang-format off
  A << 0, 1,
       0, 0;
  // clang-format on
  Eigen::Vector2d B(1, 0);
  Eigen::Matrix2d Q = Eigen::Matrix2d::Identity();
  Eigen::Matrix<double, 1, 1> R(1);
  DRAKE_EXPECT_THROWS_MESSAGE(ContinuousAlgebraicRiccatiEquation(A, B, Q, R),
                              "ContinuousAlgebraicRiccatiEquation fails.*");

  // (A, B) is not stabilizable.
  // clang-format off
  A << 1, -1,
       0, 2;
  // clang-format on
  B << 1, -1;
  DRAKE_EXPECT_THROWS_MESSAGE(ContinuousAlgebraicRiccatiEquation(A, B, Q, R),
                              "ContinuousAlgebraicRiccatiEquation fails. The "
                              "system is not stabilizable.");

  // The pair (A, B) is stabilizable (but not controllable), so the Riccati
  // equation should have a solution.
  // clang-format off
  A << -11, 30,
       -4, 11;
  // clang-format on
  B << 10, 4;
  SolveCAREandVerify(A, B, Q, R, true /* is_X_pd */);
}
}  // namespace
}  // namespace math
}  // namespace drake
