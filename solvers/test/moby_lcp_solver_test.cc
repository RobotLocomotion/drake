#include "drake/solvers/moby_lcp_solver.h"

#include <limits>
#include <memory>
#include <vector>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"

namespace drake {
namespace solvers {
namespace {

const double epsilon = 1e-6;

/// Run all non-regularized solvers.  If @p expected_z is an empty
/// vector, outputs will only be compared against each other.
template <typename Derived>
void RunBasicLcp(const Eigen::MatrixBase<Derived>& M, const Eigen::VectorXd& q,
                 const Eigen::VectorXd& expected_z_in, bool expect_fast_pass) {
  MobyLcpSolver l;

  Eigen::VectorXd expected_z = expected_z_in;

  // NOTE: We don't necessarily expect the unregularized fast solver to succeed,
  //       hence we don't test the result.
  Eigen::VectorXd fast_z;
  bool result = l.SolveLcpFast(M, q, &fast_z);
  EXPECT_GT(l.get_num_pivots(), 0);
  if (expected_z.size() == 0) {
    ASSERT_TRUE(expect_fast_pass)
        << "Expected Z not provided and expect_fast_pass unset.";
    expected_z = fast_z;
  } else {
    if (expect_fast_pass) {
      EXPECT_TRUE(CompareMatrices(fast_z, expected_z, epsilon,
                                  MatrixCompareType::absolute));
    } else {
      EXPECT_FALSE(CompareMatrices(fast_z, expected_z, epsilon,
                                   MatrixCompareType::absolute));
    }
  }

  Eigen::VectorXd lemke_z;
  result = l.SolveLcpLemke(M, q, &lemke_z);
  EXPECT_TRUE(result);
  EXPECT_TRUE(CompareMatrices(lemke_z, expected_z, epsilon,
                              MatrixCompareType::absolute));
  EXPECT_GT(l.get_num_pivots(), 0);
}

/// Run all regularized solvers.  If @p expected_z is an empty
/// vector, outputs will only be compared against each other.
template <typename Derived>
void RunRegularizedLcp(const Eigen::MatrixBase<Derived>& M,
                       const Eigen::VectorXd& q,
                       const Eigen::VectorXd& expected_z_in,
                       bool expect_fast_pass) {
  MobyLcpSolver l;

  Eigen::VectorXd expected_z = expected_z_in;

  Eigen::VectorXd fast_z;
  bool result = l.SolveLcpFastRegularized(M, q, &fast_z);
  if (expected_z.size() == 0) {
    expected_z = fast_z;
  } else {
    if (expect_fast_pass) {
      ASSERT_TRUE(result);
      EXPECT_TRUE(CompareMatrices(fast_z, expected_z, epsilon,
                                  MatrixCompareType::absolute));
    } else {
      EXPECT_FALSE(CompareMatrices(fast_z, expected_z, epsilon,
                                   MatrixCompareType::absolute));
    }
  }

  Eigen::VectorXd lemke_z;
  result = l.SolveLcpLemkeRegularized(M, q, &lemke_z);
  EXPECT_TRUE(CompareMatrices(lemke_z, expected_z, epsilon,
                              MatrixCompareType::absolute));
}

/// Run all solvers.  If @p expected_z is an empty
/// vector, outputs will only be compared against each other.
template <typename Derived>
void runLCP(const Eigen::MatrixBase<Derived>& M, const Eigen::VectorXd& q,
            const Eigen::VectorXd& expected_z_in, bool expect_fast_pass) {
  RunBasicLcp(M, q, expected_z_in, expect_fast_pass);
  RunRegularizedLcp(M, q, expected_z_in, expect_fast_pass);
}

GTEST_TEST(testMobyLCP, testTrivial) {
  Eigen::Matrix<double, 9, 9> M;
  // clang-format off
  M <<
      1, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 2, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 3, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 4, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 5, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 6, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 7, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 8, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 9;
  // clang-format on

  Eigen::Matrix<double, 9, 1> q;
  q << -1, -1, -1, -1, -1, -1, -1, -1, -1;

  Eigen::VectorXd empty_z;
  RunBasicLcp(M, q, empty_z, true);

  // Mangle the input matrix so that some regularization occurs.
  M(0, 8) = 10;
  RunRegularizedLcp(M, q, empty_z, true);
}

GTEST_TEST(testMobyLCP, testProblem1) {
  // Problem from example 10.2.1 in "Handbook of Test Problems in
  // Local and Global Optimization".
  Eigen::Matrix<double, 16, 16> M;
  M.setIdentity();
  for (int i = 0; i < M.rows() - 1; i++) {
    for (int j = i + 1; j < M.cols(); j++) {
      M(i, j) = 2;
    }
  }

  Eigen::Matrix<double, 1, 16> q;
  q.fill(-1);

  Eigen::Matrix<double, 1, 16> z;
  z.setZero();
  z(15) = 1;
  runLCP(M, q, z, false);
}

GTEST_TEST(testMobyLCP, testProblem2) {
  // Problem from example 10.2.2 in "Handbook of Test Problems in
  // Local and Global Optimization".
  Eigen::Matrix<double, 2, 2> M;
  M.fill(1);

  Eigen::Matrix<double, 1, 2> q;
  q.fill(-1);

  // This problem also has valid solutions (0, 1) and (0.5, 0.5).
  Eigen::Matrix<double, 1, 2> z;
  z << 1, 0;

  runLCP(M, q, z, true);
}

GTEST_TEST(testMobyLCP, testProblem3) {
  // Problem from example 10.2.3 in "Handbook of Test Problems in
  // Local and Global Optimization".
  Eigen::Matrix<double, 3, 3> M;

  // clang-format off
  M <<
      0, -1,  2,
      2,  0, -2,
      -1, 1,  0;
  // clang-format on

  Eigen::Matrix<double, 1, 3> q;
  q << -3, 6, -1;

  Eigen::Matrix<double, 1, 3> z;
  z << 0, 1, 3;

  runLCP(M, q, z, true);
}

GTEST_TEST(testMobyLCP, testProblem4) {
  // Problem from example 10.2.4 in "Handbook of Test Problems in
  // Local and Global Optimization".
  Eigen::Matrix<double, 4, 4> M;

  // clang-format off
  M <<
      0,  0,  10,  20,
      0,  0,  30,  15,
      10, 20,  0,   0,
      30, 15,  0,   0;
  // clang-format on

  Eigen::Matrix<double, 1, 4> q;
  q.fill(-1);

  // This solution is the third in the book, which it explicitly
  // states cannot be found using the Lemke-Howson algorithm.
  Eigen::VectorXd z(4);
  z << 1. / 90., 2. / 45., 1. / 90., 2. / 45.;

  MobyLcpSolver l;

  Eigen::VectorXd fast_z;
  bool result = l.SolveLcpFast(M, q, &fast_z);
  EXPECT_TRUE(CompareMatrices(fast_z, z, epsilon, MatrixCompareType::absolute));

  // TODO(sammy-tri) the Lemke solvers find no solution at all, however.
  fast_z.setZero();
  result = l.SolveLcpLemke(M, q, &fast_z);
  EXPECT_FALSE(result);
}

GTEST_TEST(testMobyLCP, testProblem6) {
  // Problem from example 10.2.9 in "Handbook of Test Problems in
  // Local and Global Optimization".
  Eigen::Matrix<double, 4, 4> M;

  // clang-format off
  M <<
      11,  0, 10,  -1,
      0,  11, 10,  -1,
      10, 10, 21,  -1,
      1,   1,  1,   0;  // Note that the (3, 3) position is incorrectly
                        // shown in the book with the value 1.
  // clang-format on

  // Pick a couple of arbitrary points in the [0, 23] range.
  for (double l = 1; l <= 23; l += 15) {
    Eigen::Matrix<double, 1, 4> q;
    q << 50, 50, l, -6;

    Eigen::Matrix<double, 1, 4> z;

    // clang-format off
    z << (l + 16.) / 13.,
         (l + 16.) / 13.,
         (2. * (23 - l)) / 13.,
         (1286. - (9. * l)) / 13;
    // clang-format on

    runLCP(M, q, z, true);
  }

  // Try again with a value > 23 and see that we've hit the limit as
  // described.  The fast solver has stopped working in this case
  // without regularization.
  Eigen::Matrix<double, 1, 4> q;
  q << 50, 50, 100, -6;

  Eigen::Matrix<double, 1, 4> z;
  z << 3, 3, 0, 83;

  runLCP(M, q, z, true);
}

GTEST_TEST(testMobyLCP, testEmpty) {
  Eigen::MatrixXd empty_M(0, 0);
  Eigen::VectorXd empty_q(0);
  Eigen::VectorXd z;
  MobyLcpSolver l;

  bool result = l.SolveLcpFast(empty_M, empty_q, &z);
  EXPECT_TRUE(result);
  EXPECT_EQ(z.size(), 0);

  result = l.SolveLcpLemke(empty_M, empty_q, &z);
  EXPECT_TRUE(result);
  EXPECT_EQ(z.size(), 0);

  result = l.SolveLcpFastRegularized(empty_M, empty_q, &z);
  EXPECT_TRUE(result);
  EXPECT_EQ(z.size(), 0);

  result = l.SolveLcpLemkeRegularized(empty_M, empty_q, &z);
  EXPECT_TRUE(result);
  EXPECT_EQ(z.size(), 0);
}

// Verifies that z is zero on LCP solver failure.
GTEST_TEST(testMobyLCP, testFailure) {
  Eigen::MatrixXd neg_M(1, 1);
  Eigen::VectorXd neg_q(1);

  // This LCP is unsolvable: -z - 1 cannot be greater than zero when z is
  // restricted to be non-negative.
  neg_M(0, 0) = -1;
  neg_q[0] = -1;
  Eigen::VectorXd z;
  MobyLcpSolver l;

  bool result = l.SolveLcpFast(neg_M, neg_q, &z);
  EXPECT_FALSE(result);
  ASSERT_EQ(z.size(), neg_q.size());
  EXPECT_EQ(z[0], 0.0);
  LinearComplementarityConstraint constraint(neg_M, neg_q);
  EXPECT_FALSE(constraint.CheckSatisfied(z));

  result = l.SolveLcpLemke(neg_M, neg_q, &z);
  EXPECT_FALSE(result);
  ASSERT_EQ(z.size(), neg_q.size());
  EXPECT_EQ(z[0], 0.0);
  EXPECT_FALSE(constraint.CheckSatisfied(z));

  // Note: we do not test regularized solvers here, as we're specifically
  // interested in algorithm failure and the regularized solvers are designed
  // not to fail.
}

}  // namespace
}  // namespace solvers
}  // namespace drake
