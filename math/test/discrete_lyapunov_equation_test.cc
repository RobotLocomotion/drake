#include "drake/math/discrete_lyapunov_equation.h"

#include <limits>
#include <stdexcept>
#include <vector>

#include <Eigen/Core>
#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_no_throw.h"

namespace drake {
namespace math {
namespace {

const double kEps = std::numeric_limits<double>::epsilon();

using Eigen::MatrixXd;

const double kTolerance = 5 * std::numeric_limits<double>::epsilon();

void SolveRealLyapunovEquationAndVerify(const Eigen::Ref<const MatrixXd>& A,
                                        const Eigen::Ref<const MatrixXd>& Q,
                                        double symmetry_tol = 0) {
  MatrixXd X{RealDiscreteLyapunovEquation(A, Q)};
  // Check that X is symmetric.
  EXPECT_TRUE(CompareMatrices(X, X.transpose(), symmetry_tol,
                              MatrixCompareType::absolute));
  // Check that X is the solution to the discrete time ARE.
  EXPECT_TRUE(CompareMatrices(A.transpose() * X * A - X, -Q,
                              5 * kTolerance * Q.norm(),
                              MatrixCompareType::absolute));
}

GTEST_TEST(RealDiscreteLyapunovEquation, ThrowInvalidSizedMatricesTest) {
  // This tests if the exceptions are thrown for invalidly sized matrices. A and
  // Q need to be square and of same size.
  const int n{1}, m{2};

  // non-square A
  MatrixXd A1(n, m);
  A1 << 1, 1;
  MatrixXd Q1(m, m);
  Q1 << 1, 1, 1, 1;

  // non-square Q
  MatrixXd A2(m, m);
  A2 << 1, 1, 1, 1;
  MatrixXd Q2(n, m);
  Q2 << 1, 1;

  // not-same sized
  MatrixXd A3(m, m);
  A3 << 1, 1, 1, 1;
  MatrixXd Q3(n, n);
  Q3 << 1;

  std::vector<MatrixXd> As{A1, A2, A3};
  std::vector<MatrixXd> Qs{Q1, Q2, Q3};

  for (int i = 0; i < static_cast<int>(As.size()); ++i) {
    EXPECT_ANY_THROW(RealDiscreteLyapunovEquation(As[i], Qs[i]));
  }
}

GTEST_TEST(RealDiscreteLyapunovEquation, ThrowEigenValuesATest) {
  // Given the Eigenvalues of @param A as lambda_1, ..., lambda_n, then the
  // solution is unique if and only if lambda_i * lambda_j != 1 for all  i, j.
  // (see Barraud, A.Y., "A numerical algorithm to solve AᵀXA - X = Q," IEEE®
  // Trans. Auto. Contr., AC-22, pp. 883-885, 1977.)
  // This tests if an exception is thrown if the eigenvalues violate this
  // requirement.
  const int n{3};
  // pair of eigenvalues that multiplies to 1
  MatrixXd A1(n, n);
  A1 << 1, 0, 0, 0, 1, 0, 0, 0, 1;
  // pairs of eigenvalues that multiply to ±i, i.e should not throw
  MatrixXd A2(n, n);
  A2 << 0.5, 0, 0, 0, 0, 2.0, 0, -2.0, 0;
  // pair of eigenvalues whose product is within tol of 1
  MatrixXd A3(n, n);
  A3 << 1 + 1e-6, 0, 0, 0, 0.5, 0, 0, 0, 1 - 1e-6;
  // no pair of eigenvalues whose product is within tol of 1
  MatrixXd A4(n, n);
  A4 << 1 + 1e-6, 0, 0, 0, 1 + 1e-6, 0, 0, 0, 1 + 1e-6;
  MatrixXd Q(n, n);
  Q << 1, 0, 0, 0, 1, 0, 0, 0, 1;

  EXPECT_ANY_THROW(RealDiscreteLyapunovEquation(A1, Q));
  DRAKE_EXPECT_NO_THROW(RealDiscreteLyapunovEquation(A2, Q));
  EXPECT_ANY_THROW(RealDiscreteLyapunovEquation(A3, Q));
  DRAKE_EXPECT_NO_THROW(RealDiscreteLyapunovEquation(A4, Q));
}

GTEST_TEST(RealDiscreteLyapunovEquation, Solve1by1Test) {
  // This is a simple 1-by-1 test case, it tests the internal 1-by-1 solver.
  const int n{1};
  MatrixXd A(n, n);
  A << 0.5;
  MatrixXd Q(n, n);
  Q << 1;
  MatrixXd X(n, n);
  X << 4.0 / 3.0;

  EXPECT_TRUE(
      CompareMatrices(internal::Solve1By1RealDiscreteLyapunovEquation(A, Q), X,
                      kTolerance, MatrixCompareType::absolute));

  SolveRealLyapunovEquationAndVerify(A, Q);
}

GTEST_TEST(RealDiscreteLyapunovEquation, Solve2by2Test) {
  // This is a simple 2-by-2 test case, which tests the internal 2-by-2 solver.
  // The result is compared to the one generated by Matlab's dlyap function.
  const int n{2};
  MatrixXd A(n, n);
  A << 0.5, -0.5, 0, 0.25;
  MatrixXd X(n, n);
  X << 4.0, 0, 0, 2.0 + 1.0 / 7.5;

  MatrixXd Q_internal(n, n);
  Q_internal << 3, 1, NAN, 1;
  EXPECT_TRUE(CompareMatrices(
      internal::Solve2By2RealDiscreteLyapunovEquation(A, Q_internal), X,
      kTolerance, MatrixCompareType::absolute));

  MatrixXd Q(n, n);
  Q << 3, 1, 1, 1;
  EXPECT_TRUE(CompareMatrices(RealDiscreteLyapunovEquation(A, Q), X, kTolerance,
                              MatrixCompareType::absolute));
  SolveRealLyapunovEquationAndVerify(A.transpose(), Q);
}

GTEST_TEST(RealDiscreteLyapunovEquation, Solve3by3Test1) {
  // Tests if a 3-by-3 problem is reduced.
  const int n{3};
  MatrixXd A(n, n);
  A << -0.5 * MatrixXd::Identity(n, n);
  MatrixXd Q(n, n);
  Q << MatrixXd::Identity(n, n);

  SolveRealLyapunovEquationAndVerify(A, Q);
}

GTEST_TEST(RealDiscreteLyapunovEquation, Solve3by3Test2) {
  // The system has eigenvalues: lambda_1/2  = -0 +/- 0.5i
  // and lambda_3 = -0.5. Therefore, there exists a 2-by-2 block on the
  // diagonal.
  // The compared solution is generated by matlab's dlyap function.

  int n{3};
  MatrixXd A(n, n);
  A << 0.5, 0, 0, 0, 0, 0.5, 0, -0.5, 0;
  MatrixXd Q(n, n);
  Q << MatrixXd::Identity(n, n);
  MatrixXd X(n, n);
  X << (4.0 / 3.0) * MatrixXd::Identity(n, n);

  EXPECT_TRUE(CompareMatrices(RealDiscreteLyapunovEquation(A, Q), X, kTolerance,
                              MatrixCompareType::absolute));
  SolveRealLyapunovEquationAndVerify(A, Q);
}

GTEST_TEST(RealDiscreteLyapunovEquation, Solve3by3Test3) {
  int n{3};
  MatrixXd A(n, n);
  A << 0.5, 1, -1, 1, 0, 0.5, 0, -0.5, 2;
  MatrixXd Q(n, n);
  // clang-format off
  Q << 10, 2, 3,
       2, 4, 1,
       3, 1, 5;
  // clang-format on
  SolveRealLyapunovEquationAndVerify(A, Q, 100 * kEps);
}

GTEST_TEST(RealDiscreteLyapunovEquation, Solve4by4Test1) {
  // The system has eigenvalues: lambda_1/2  = -0.4500 +/- 0.7794i
  // and lambda_3/4 = -0.9.

  const int n{4};
  MatrixXd A(n, n);
  A << -0.9, 0, 0, 0, 0, 0, 0.9, 0, 0, -0.9, -0.9, 0, 0, 0, 0, -0.9;
  MatrixXd Q(n, n);
  Q << MatrixXd::Identity(n, n);
  SolveRealLyapunovEquationAndVerify(A, Q);
}

GTEST_TEST(RealDiscreteLyapunovEquation, Solve4by4Test2) {
  // The system has eigenvalues: lambda_1/2  = 0.3 +/- 0.4i
  // and lambda_3/4 = 0.4 +/- 0.5i

  const int n{4};
  MatrixXd A(n, n);
  A << 0.3, 0, 0, 0.4, 0, 0.4, 0.5, 0, 0, -0.5, 0.4, 0, -0.4, 0, 0, 0.3;
  MatrixXd Q(n, n);
  Q << MatrixXd::Identity(n, n);
  SolveRealLyapunovEquationAndVerify(A, Q);
}

}  // namespace
}  // namespace math
}  // namespace drake
