#include "drake/solvers/unrevised_lemke_solver.h"

#include <memory>
#include <vector>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"

namespace drake {
namespace solvers {
namespace {

// Run the solver and test against the expected result.
template <typename Derived>
void runLCP(const Eigen::MatrixBase<Derived>& M, const Eigen::VectorXd& q,
                 const Eigen::VectorXd& expected_z_in) {
  UnrevisedLemkeSolver<double> l;

  Eigen::VectorXd expected_z = expected_z_in;
  Eigen::VectorXd lemke_z;
  int num_pivots;
  bool result = l.SolveLcpLemke(M, q, &lemke_z, &num_pivots);

  // Result should be false, since the solver is still stubbed.
  ASSERT_FALSE(result);
}

GTEST_TEST(testUnrevisedLCP, testCycling) {
  Eigen::Matrix<double, 3, 3> M;

  // clang-format off
  M <<
    1, 2, 0,
    0, 1, 2,
    2, 0, 1;
  // clang-format on

  Eigen::Matrix<double, 3, 1> q;
  q << -1, -1, -1;

  Eigen::VectorXd expected_z(3);
  expected_z << 1.0/3, 1.0/3, 1.0/3;
  runLCP(M, q, expected_z);
}

GTEST_TEST(testUnrevisedLCP, testTrivial) {
  Eigen::Matrix<double, 9, 9> M;
  M = (Eigen::Matrix<double, 9, 1>() << 1, 2, 3, 4, 5, 6, 7, 8, 9).
      finished().asDiagonal();

  Eigen::Matrix<double, 9, 1> q;
  q << -1, -1, -1, -1, -1, -1, -1, -1, -1;

  Eigen::VectorXd expected_z(9);
  expected_z << 1, 1.0/2, 1.0/3, 1.0/4, 1.0/5, 1.0/6, 1.0/7, 1.0/8, 1.0/9;
  runLCP(M, q, expected_z);
}

GTEST_TEST(testUnrevisedLCP, testProblem1) {
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
  runLCP(M, q, z);
}

GTEST_TEST(testUnrevisedLCP, testProblem2) {
  // Problem from example 10.2.2 in "Handbook of Test Problems in
  // Local and Global Optimization".
  Eigen::Matrix<double, 2, 2> M;
  M.fill(1);

  Eigen::Matrix<double, 1, 2> q;
  q.fill(-1);

  // This problem also has valid solutions (0, 1) and (0.5, 0.5).
  Eigen::Matrix<double, 1, 2> z;
  z << 1, 0;

  runLCP(M, q, z);
}

GTEST_TEST(testUnrevisedLCP, testProblem3) {
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

  runLCP(M, q, z);
}

GTEST_TEST(testUnrevisedLCP, testProblem4) {
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

  UnrevisedLemkeSolver<double> l;

  // The Lemke solvers find no solution at all.
  int num_pivots;
  bool result = l.SolveLcpLemke(M, q, &z, &num_pivots);
  EXPECT_FALSE(result);
}

GTEST_TEST(testUnrevisedLCP, testProblem6) {
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

    runLCP(M, q, z);
  }

  // Try again with a value > 23 and see that we've hit the limit as
  // described.  The fast solver has stopped working in this case
  // without regularization.
  Eigen::Matrix<double, 1, 4> q;
  q << 50, 50, 100, -6;

  Eigen::Matrix<double, 1, 4> z;
  z << 3, 3, 0, 83;

  runLCP(M, q, z);
}

GTEST_TEST(testUnrevisedLCP, testEmpty) {
  Eigen::MatrixXd empty_M(0, 0);
  Eigen::VectorXd empty_q(0);
  Eigen::VectorXd z;
  UnrevisedLemkeSolver<double> l;

  int num_pivots;
  bool result = l.SolveLcpLemke(empty_M, empty_q, &z, &num_pivots);
  EXPECT_FALSE(result);
  EXPECT_EQ(z.size(), 0);
}

// Verifies that z is zero on LCP solver failure.
GTEST_TEST(testUnrevisedLCP, testFailure) {
  Eigen::MatrixXd neg_M(1, 1);
  Eigen::VectorXd neg_q(1);

  // This LCP is unsolvable: -z - 1 cannot be greater than zero when z is
  // restricted to be non-negative.
  neg_M(0, 0) = -1;
  neg_q[0] = -1;
  Eigen::VectorXd z;
  UnrevisedLemkeSolver<double> l;

  int num_pivots;
  bool result = l.SolveLcpLemke(neg_M, neg_q, &z, &num_pivots);
  LinearComplementarityConstraint constraint(neg_M, neg_q);
  EXPECT_FALSE(result);
  ASSERT_EQ(z.size(), neg_q.size());
  EXPECT_EQ(z[0], 0.0);
  EXPECT_FALSE(constraint.CheckSatisfied(z));
}

}  // namespace
}  // namespace solvers
}  // namespace drake
