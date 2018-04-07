#include "drake/solvers/unrevised_lemke_solver.h"

#include <memory>
#include <vector>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"

namespace drake {
namespace solvers {

const double epsilon = 5e-14;

// Run the solver and test against the expected result.
template <typename Derived>
void RunLCP(const Eigen::MatrixBase<Derived>& M, const Eigen::VectorXd& q,
            const Eigen::VectorXd& expected_z_in) {
  UnrevisedLemkeSolver<double> l;

  Eigen::VectorXd expected_z = expected_z_in;

  // NOTE: We don't necessarily expect the unregularized fast solver to succeed,
  //       hence we don't test the result.
  Eigen::VectorXd lemke_z;
  int num_pivots;
  bool result = l.SolveLcpLemke(M, q, &lemke_z, &num_pivots);
  ASSERT_TRUE(result);
  EXPECT_TRUE(CompareMatrices(lemke_z, expected_z, epsilon,
                              MatrixCompareType::absolute));
  EXPECT_GT(num_pivots, 0);  // We do not test any trivial LCPs.
}

GTEST_TEST(TestUnrevisedLemke, TestCycling) {
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
  RunLCP(M, q, expected_z);
}

GTEST_TEST(TestUnrevisedLemke, TestTrivial) {
  // Create a 9x9 diagonal matrix from the vector [1 2 3 4 5 6 7 8 9].
  MatrixX<double> M = (Eigen::Matrix<double, 9, 1>() <<
      1, 2, 3, 4, 5, 6, 7, 8, 9).finished().asDiagonal();

  Eigen::Matrix<double, 9, 1> q;
  q << -1, -1, -1, -1, -1, -1, -1, -1, -1;

  Eigen::VectorXd expected_z(9);
  expected_z << 1, 1.0/2, 1.0/3, 1.0/4, 1.0/5, 1.0/6, 1.0/7, 1.0/8, 1.0/9;
  RunLCP(M, q, expected_z);
}

GTEST_TEST(TestUnrevisedLemke, TestProblem1) {
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
  RunLCP(M, q, z);
}

GTEST_TEST(TestUnrevisedLemke, TestProblem2) {
  // Problem from example 10.2.2 in "Handbook of Test Problems in
  // Local and Global Optimization".
  Eigen::Matrix<double, 2, 2> M;
  M.fill(1);

  Eigen::Matrix<double, 1, 2> q;
  q.fill(-1);

  // This problem also has valid solutions (0, 1) and (0.5, 0.5).
  Eigen::Matrix<double, 1, 2> z;
  z << 1, 0;

  RunLCP(M, q, z);
}

GTEST_TEST(TestUnrevisedLemke, TestProblem3) {
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

  RunLCP(M, q, z);
}

GTEST_TEST(TestUnrevisedLemke, TestProblem4) {
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
  int num_pivots;
  bool result = l.SolveLcpLemke(M, q, &z, &num_pivots);
  EXPECT_FALSE(result);
}

GTEST_TEST(TestUnrevisedLemke, TestProblem6) {
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

    RunLCP(M, q, z);
  }

  // Try again with a value > 23 and see that we've hit the limit as
  // described.  The fast solver has stopped working in this case
  // without regularization.
  Eigen::Matrix<double, 1, 4> q;
  q << 50, 50, 100, -6;

  Eigen::Matrix<double, 1, 4> z;
  z << 3, 3, 0, 83;

  RunLCP(M, q, z);
}

GTEST_TEST(TestUnrevisedLemke, TestEmpty) {
  Eigen::MatrixXd empty_M(0, 0);
  Eigen::VectorXd empty_q(0);
  Eigen::VectorXd z;
  UnrevisedLemkeSolver<double> l;

  int num_pivots;
  bool result = l.SolveLcpLemke(empty_M, empty_q, &z, &num_pivots);
  EXPECT_TRUE(result);
  EXPECT_EQ(z.size(), 0);
}

// Verifies that z is zero on LCP solver failure.
GTEST_TEST(TestUnrevisedLemke, TestFailure) {
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

GTEST_TEST(TestUnrevisedLemke, TestSolutionQuality) {
  // Set the LCP and the solution.
  VectorX<double> q(1), z(1);
  MatrixX<double> M(1, 1);
  M(0, 0) = 1;
  q[0] = -1;
  z[0] = 1 - std::numeric_limits<double>::epsilon();

  // Check solution quality without a tolerance specified.
  UnrevisedLemkeSolver<double> lcp;
  EXPECT_TRUE(lcp.IsSolution(M, q, z));

  // Check solution quality with a tolerance specified.
  EXPECT_TRUE(lcp.IsSolution(M, q, z, 3e-16));
}

GTEST_TEST(TestUnrevisedLemke, ZeroTolerance) {
  // Compute the zero tolerance for several matrices.
  // An scalar matrix- should be _around_ machine epsilon.
  const double eps = std::numeric_limits<double>::epsilon();
  MatrixX<double> M(1, 1);
  M(0, 0) = 1;
  EXPECT_NEAR(UnrevisedLemkeSolver<double>::ComputeZeroTolerance(M), eps,
              10 * eps);

  // An scalar matrix * 1e10. Should be _around_ machine epsilon * 1e10.
  M(0, 0) = 1e10;
  EXPECT_NEAR(UnrevisedLemkeSolver<double>::ComputeZeroTolerance(M),
              1e10 * eps, 1e11 * eps);

  // A 100 x 100 identity matrix. Should be _around_ 100 * machine epsilon.
  M = MatrixX<double>::Identity(10, 10);
  EXPECT_NEAR(UnrevisedLemkeSolver<double>::ComputeZeroTolerance(M), 1e2 * eps,
              1e3 * eps);
}

// Tests whether warm starting works properly.
GTEST_TEST(TestUnrevisedLemke, WarmStarting) {
}

// A class for testing various private functions in the Lemke solver.
class UnrevisedLemkePrivateTests : public testing::Test {
 protected:
  UnrevisedLemkeSolver<double> lcp_;
};

// Tests proper operation of selecting a sub-matrix from a matrix that is
// augmented with a covering vector.
TEST_F(UnrevisedLemkePrivateTests, SelectSubMatrixWithCovering) {
  MatrixX<double> result;

  // After augmentation, the matrix will be:
  // 1 0 0 1
  // 0 1 0 1
  // 0 0 1 1
  MatrixX<double> M = MatrixX<double>::Identity(3, 3);

  // Select the upper-left 2x2
  lcp_.SelectSubMatrixWithCovering(M, {0, 1}, {0, 1}, &result);
  ASSERT_EQ(result.rows(), 2);
  ASSERT_EQ(result.cols(), 2);
  MatrixX<double> expected(result.rows(), result.cols());
  expected << 1, 0,
              0, 1;
  EXPECT_TRUE(CompareMatrices(result, expected, epsilon,
                              MatrixCompareType::absolute));

  // Select the lower-right 2x2.
  lcp_.SelectSubMatrixWithCovering(M, {1, 2}, {2, 3}, &result);
  ASSERT_EQ(result.rows(), 2);
  ASSERT_EQ(result.cols(), 2);
  expected << 0, 1,
              1, 1;
  EXPECT_TRUE(CompareMatrices(result, expected, epsilon,
                              MatrixCompareType::absolute));

  // Select the right 3x3, with columns reversed.
  lcp_.SelectSubMatrixWithCovering(M, {0, 1, 2}, {3, 2, 1}, &result);
  ASSERT_EQ(result.rows(), 3);
  ASSERT_EQ(result.cols(), 3);
  expected = MatrixX<double>(result.rows(), result.cols());
  expected << 1, 0, 0,
              1, 0, 1,
              1, 1, 0;
  EXPECT_TRUE(CompareMatrices(result, expected, epsilon,
                              MatrixCompareType::absolute));

  // Select the right 3x3, with rows reversed.
  lcp_.SelectSubMatrixWithCovering(M, {2, 1, 0}, {1, 2, 3}, &result);
  ASSERT_EQ(result.rows(), 3);
  ASSERT_EQ(result.cols(), 3);
  expected = MatrixX<double>(result.rows(), result.cols());
  expected << 0, 1, 1,
              1, 0, 1,
              0, 0, 1;
  EXPECT_TRUE(CompareMatrices(result, expected, epsilon,
                              MatrixCompareType::absolute));

  // Select the entire matrix.
  lcp_.SelectSubMatrixWithCovering(M, {0, 1, 2}, {0, 1, 2, 3}, &result);
  expected = MatrixX<double>(result.rows(), result.cols());
  expected << 1, 0, 0, 1,
              0, 1, 0, 1,
              0, 0, 1, 1;
  EXPECT_TRUE(CompareMatrices(result, expected, epsilon,
                              MatrixCompareType::absolute));
}

// Tests proper operation of selecting a sub-column from a matrix that is
// augmented with a covering vector.
TEST_F(UnrevisedLemkePrivateTests,SelectSubColumnWithCovering) {
}

// Tests proper operation of selecting a sub-vector from a vector.
TEST_F(UnrevisedLemkePrivateTests,SelectSubVector) {
  // Set the vector.
  VectorX<double> v(3);
  v << 0, 1, 2;

  // One element (the middle one).
  VectorX<double> result;
  lcp_.SelectSubVector(v, { 1 }, &result);
  EXPECT_EQ(result.size(), 1);
  EXPECT_EQ(result[0], 1);

  // Two elements (the ends).
  lcp_.SelectSubVector(v, { 0, 2 }, &result);
  EXPECT_EQ(result.size(), 2);
  EXPECT_EQ(result[0], 0);
  EXPECT_EQ(result[1], 2);

  // All three elements, not ordered sequentially.
  lcp_.SelectSubVector(v, { 0, 2, 1 }, &result);
  EXPECT_EQ(result.size(), 3);
  EXPECT_EQ(result[0], 0);
  EXPECT_EQ(result[1], 2);
  EXPECT_EQ(result[2], 1);
}

TEST_F(UnrevisedLemkePrivateTests, SetSubVector) {
  // Construct a zero vector that will be used repeatedly for reinitialization.
  VectorX<double> zero(3);
  zero << 0, 0, 0;

  // Set a single element.
  VectorX<double> result = zero;
  VectorX<double> v_sub(1);
  v_sub << 1;
  lcp_.SetSubVector(v_sub, {0}, &result);
  EXPECT_EQ(result[0], 1.0);
  EXPECT_EQ(result.norm(), 1.0);  // Verify no other elements were set.

  // Set another single element, this time at the end.
  result = zero;
  lcp_.SetSubVector(v_sub, {2}, &result);
  EXPECT_EQ(result[2], 1.0);
  EXPECT_EQ(result.norm(), 1.0);  // Verify no other elements were set.

  // Set two elements, one at either end.
  v_sub = VectorX<double>(2);
  v_sub << 2, 3;
  lcp_.SetSubVector(v_sub, {0, 2}, &result);
  EXPECT_EQ(result[0], 2);
  EXPECT_EQ(result[1], 0);
  EXPECT_EQ(result[2], 3);

  // Set an entire vector, in reverse order.
  v_sub = VectorX<double>(3);
  v_sub << 1, 2, 3;
  lcp_.SetSubVector(v_sub, {2, 1, 0}, &result);
  EXPECT_EQ(result[0], 3);
  EXPECT_EQ(result[1], 2);
  EXPECT_EQ(result[2], 1);
}

TEST_F(UnrevisedLemkePrivateTests, ValidateIndices) {
}

TEST_F(UnrevisedLemkePrivateTests, IsEachUnique) {
}

TEST_F(UnrevisedLemkePrivateTests, LemkePivot) {
}

TEST_F(UnrevisedLemkePrivateTests, ConstructLemkeSolution) {
}

TEST_F(UnrevisedLemkePrivateTests, DetermineIndexSets) {
}

TEST_F(UnrevisedLemkePrivateTests, FindBlockingIndex) {
}


}  // namespace solvers
}  // namespace drake
