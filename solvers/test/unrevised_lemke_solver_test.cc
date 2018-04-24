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

// Checks that the solver detects a dimensional mismatch between the LCP matrix
// and vector.
GTEST_TEST(TestUnrevisedLemke, DimensionalMismatch) {
  UnrevisedLemkeSolver<double> lcp;
  MatrixX<double> M(3, 3);
  VectorX<double> q(4);

  // Zero tolerance is arbitrary.
  const double zero_tol = 1e-15;

  // Verify that solver catches M not matching q.
  int num_pivots;
  VectorX<double> z;
  EXPECT_THROW(lcp.SolveLcpLemke(M, q, &z, &num_pivots, zero_tol),
               std::logic_error);

  // Verify that solver catches non-square M.
  M.resize(3, 4);
  EXPECT_THROW(lcp.SolveLcpLemke(M, q, &z, &num_pivots, zero_tol),
               std::logic_error);
}

// Checks the robustness of the algorithm to a known test problem that results
// in cycling.
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

// Tests a simple linear complementarity problem.
GTEST_TEST(TestUnrevisedLemke, TestSimple) {
  // Create a 9x9 diagonal matrix from the vector [1 2 3 4 5 6 7 8 9].
  MatrixX<double> M = (Eigen::Matrix<double, 9, 1>() <<
      1, 2, 3, 4, 5, 6, 7, 8, 9).finished().asDiagonal();

  Eigen::Matrix<double, 9, 1> q;
  q << -1, -1, -1, -1, -1, -1, -1, -1, -1;

  Eigen::VectorXd expected_z(9);
  expected_z << 1, 1.0/2, 1.0/3, 1.0/4, 1.0/5, 1.0/6, 1.0/7, 1.0/8, 1.0/9;
  RunLCP(M, q, expected_z);
}

// Tests that the artificial variable is always selected in a tie
// (Example 4.4.16 in [Cottle 1992]). We know Lemke's algorithm can solve
// this one since the matrix is symmetric and positive semi-definite.
// Lemke implementations without the necessary special-case code can terminate
// on an unblocked variable (i.e., fail to find the solution when one is known
// to exist).
// NOTE: This is a necessary but not sufficient test that the special-case code
// is working. This test failed before the special-case code was added, but it's
// possible that the test could succeed using other strategies for selecting
// one of multiple valid blocking indices. For example, Miranda and Fackler's
// Lemke solver uses a random blocking variable selection when multiple are
// possible.
GTEST_TEST(TestUnrevisedLemke, TestPSD) {
  MatrixX<double> M(2, 2);
  M << 1, -1,
      -1, 1;

  Eigen::Vector2d q;
  q << 1, -1;

  Eigen::VectorXd expected_z(2);
  expected_z << 0, 1;
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

  // Try again with a value > 23 and verify that Lemke is still successful.
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

// Checks that warmstarting works as anticipated.
GTEST_TEST(TestUnrevisedLemke, WarmStarting) {
  MatrixX<double> M(3, 3);
  // clang-format off
  M <<
      1, 2, 0,
      0, 1, 2,
      2, 0, 1;
  // clang-format on

  Eigen::Matrix<double, 3, 1> q;
  q << -1, -1, -1;

  // Solve the problem once.
  int num_pivots;
  Eigen::VectorXd expected_z(3);
  expected_z << 1.0/3, 1.0/3, 1.0/3;
  Eigen::VectorXd z;
  UnrevisedLemkeSolver<double> lcp;
  bool result = lcp.SolveLcpLemke(M, q, &z, &num_pivots);
  ASSERT_TRUE(result);
  ASSERT_TRUE(CompareMatrices(z, expected_z, epsilon,
                              MatrixCompareType::absolute));

  // Verify that more than one pivot was required.
  EXPECT_GE(num_pivots, 1);

  // Solve the problem with a slightly different q and verify that exactly
  // one pivot was required.
  q *= 2;
  expected_z *= 2;
  result = lcp.SolveLcpLemke(M, q, &z, &num_pivots);
  ASSERT_TRUE(result);
  ASSERT_TRUE(CompareMatrices(z, expected_z, epsilon,
                              MatrixCompareType::absolute));
  EXPECT_EQ(num_pivots, 1);
}

// Checks that an LCP with a trivial solution is solvable without any pivots.
GTEST_TEST(TestUnrevisedLemke, Trivial) {
  MatrixX<double> M = MatrixX<double>::Identity(3, 3);
  Eigen::Matrix<double, 3, 1> q;
  q << 1, 1, 1;

  // Solve the problem.
  int num_pivots;
  Eigen::VectorXd expected_z(3);
  expected_z << 0, 0, 0;
  Eigen::VectorXd z;
  UnrevisedLemkeSolver<double> lcp;
  bool result = lcp.SolveLcpLemke(M, q, &z, &num_pivots);
  ASSERT_TRUE(result);
  ASSERT_TRUE(CompareMatrices(z, expected_z, epsilon,
                              MatrixCompareType::absolute));
  ASSERT_EQ(num_pivots, 0);
}

// A class for testing various private functions in the Lemke solver.
class UnrevisedLemkePrivateTests : public testing::Test {
 protected:
  void SetUp() {
    typedef UnrevisedLemkeSolver<double>::LCPVariable LCPVariable;

    // clang-format off
    M_.resize(3, 3);
    M_ <<
        0, -1,  2,
        2,  0, -2,
        -1, 1,  0;
    // clang-format on

    q_.resize(3, 1);
    q_ << -3, 6, -1;

    // Set the LCP variables. Start with all z variables independent and all w
    // variables dependent.
    const int n = 3;
    lcp_.indep_variables_.resize(n+1);
    lcp_.dep_variables_.resize(n);
    for (int i = 0; i < n; ++i) {
      lcp_.dep_variables_[i] = LCPVariable(false, i);
      lcp_.indep_variables_[i] = LCPVariable(true, i);
    }
    // z needs one more variable (the artificial variable), whose index we
    // denote as n to keep it from corresponding to any actual vector index.
    lcp_.indep_variables_[n] = LCPVariable(true, n);
  }

  UnrevisedLemkeSolver<double> lcp_;  // The solver itself.
  MatrixX<double> M_;                 // The LCP matrix used in pivoting tests.
  MatrixX<double> q_;                 // The LCP vector used in pivoting tests.
  int kArtificial{3};                 // Index of the artificial variable.
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
TEST_F(UnrevisedLemkePrivateTests, SelectSubColumnWithCovering) {
  // After augmentation, the matrix will be:
  // 1 0 0 1
  // 0 1 0 1
  // 0 0 1 1
  MatrixX<double> M = MatrixX<double>::Identity(3, 3);

  VectorX<double> result;

  // Get a single row, first column.
  VectorX<double> expected(1);
  expected << 1;
  lcp_.SelectSubColumnWithCovering(M, {0}, 0 /* column */, &result);
  EXPECT_TRUE(CompareMatrices(result, expected, epsilon,
                              MatrixCompareType::absolute));

  // Get another single row from the first column.
  expected << 0;
  lcp_.SelectSubColumnWithCovering(M, {1}, 0 /* column */, &result);
  EXPECT_TRUE(CompareMatrices(result, expected, epsilon,
                              MatrixCompareType::absolute));

  // Get first and third rows in forward order, first column.
  lcp_.SelectSubColumnWithCovering(M, {0, 2}, 0 /* column */, &result);
  expected = VectorX<double>(2);
  expected << 1, 0;
  EXPECT_TRUE(CompareMatrices(result, expected, epsilon,
                              MatrixCompareType::absolute));

  // Get first and third rows in reverse order, second column.
  lcp_.SelectSubColumnWithCovering(M, {2, 0}, 1 /* column */, &result);
  expected << 0, 0;
  EXPECT_TRUE(CompareMatrices(result, expected, epsilon,
                              MatrixCompareType::absolute));

  // Get all three rows in forward order, third column.
  lcp_.SelectSubColumnWithCovering(M, {0, 1, 2}, 2 /* column */, &result);
  expected = VectorX<double>(3);
  expected << 0, 0, 1;
  EXPECT_TRUE(CompareMatrices(result, expected, epsilon,
                              MatrixCompareType::absolute));

  // Get all three rows in reverse order, third column.
  lcp_.SelectSubColumnWithCovering(M, {2, 1, 0}, 2 /* column */, &result);
  expected << 1, 0, 0;
  EXPECT_TRUE(CompareMatrices(result, expected, epsilon,
                              MatrixCompareType::absolute));

  // Get one row from the fourth column.
  lcp_.SelectSubColumnWithCovering(M, {0}, 3 /* column */, &result);
  EXPECT_EQ(result.lpNorm<1>(), 1);

  // Get two rows from the fourth column.
  lcp_.SelectSubColumnWithCovering(M, {0, 1}, 3 /* column */, &result);
  EXPECT_EQ(result.lpNorm<1>(), 2);

  // Get three rows from the fourth column.
  lcp_.SelectSubColumnWithCovering(M, {0, 1, 2}, 3 /* column */, &result);
  EXPECT_EQ(result.lpNorm<1>(), 3);
}

// Tests proper operation of selecting a sub-vector from a vector.
TEST_F(UnrevisedLemkePrivateTests, SelectSubVector) {
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

// Verifies proper operation of SetSubVector().
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

// Checks whether ValidateIndices(), which checks that a vector of indices used
// to select a sub-block of a matrix or vector is within range and unique.
TEST_F(UnrevisedLemkePrivateTests, ValidateIndices) {
  // Verifies that a proper set of indices works.
  const int first_set_size = 3;
  EXPECT_TRUE(lcp_.ValidateIndices({0, 1, 2}, first_set_size));

  // Verifies that indices need not be in sorted order.
  EXPECT_TRUE(lcp_.ValidateIndices({2, 1, 0}, first_set_size));

  // Verifies that ValidateIndices() catches a repeated index.
  EXPECT_FALSE(lcp_.ValidateIndices({0, 1, 1}, first_set_size));

  // Verifies that ValidateIndices() catches indices out of range.
  EXPECT_FALSE(lcp_.ValidateIndices({0, 1, 4}, first_set_size));
  EXPECT_FALSE(lcp_.ValidateIndices({0, 1, -1}, first_set_size));

  // ** Two-index set tests **.
  // Verifies that a proper set of indices works.
  const int second_set_size = 7;
  EXPECT_TRUE(lcp_.ValidateIndices({0, 1, 2}, {3, 4, 5, 6}, first_set_size,
                                   second_set_size));

  // Verifies that indices need not be in sorted order.
  EXPECT_TRUE(lcp_.ValidateIndices({2, 1, 0}, {6, 5, 4, 3}, first_set_size,
                                   second_set_size));

  // Verifies that ValidateIndices() catches a single repeated index.
  EXPECT_FALSE(lcp_.ValidateIndices({0, 1, 1}, {3, 4, 5, 6}, first_set_size,
                                    second_set_size));

  // Verifies that ValidateIndices() catches indices out of range.
  EXPECT_FALSE(lcp_.ValidateIndices({0, 1, 4}, {3, 4, 5, 6}, first_set_size,
                                    second_set_size));
  EXPECT_FALSE(lcp_.ValidateIndices({0, 1, -1}, {3, 4, 5, 6}, first_set_size,
                                    second_set_size));
}

// Verifies proper operation of IsEachUnique(), which checks whether LCP
// variables in a vector are unique.
TEST_F(UnrevisedLemkePrivateTests, IsEachUnique) {
  // Create two variables with the same index, but one z and one w. These should
  // be reported as unique.
  EXPECT_TRUE(lcp_.IsEachUnique(
      {UnrevisedLemkeSolver<double>::LCPVariable(true, 0),
       UnrevisedLemkeSolver<double>::LCPVariable(false, 0)}));

  // Create two variables with different indices, but both z. These should be
  // reported as unique.
  EXPECT_TRUE(lcp_.IsEachUnique(
      {UnrevisedLemkeSolver<double>::LCPVariable(true, 0),
       UnrevisedLemkeSolver<double>::LCPVariable(true, 1)}));

  // Create two variables with different indices, but both w. These should be
  // reported as unique.
  EXPECT_TRUE(lcp_.IsEachUnique(
      {UnrevisedLemkeSolver<double>::LCPVariable(false, 0),
       UnrevisedLemkeSolver<double>::LCPVariable(false, 1)}));

  // Create two identical variables. These should not be reported as unique.
  EXPECT_FALSE(lcp_.IsEachUnique(
      {UnrevisedLemkeSolver<double>::LCPVariable(false, 0),
       UnrevisedLemkeSolver<double>::LCPVariable(false, 0)}));
  EXPECT_FALSE(lcp_.IsEachUnique(
      {UnrevisedLemkeSolver<double>::LCPVariable(true, 1),
       UnrevisedLemkeSolver<double>::LCPVariable(true, 1)}));
}

// Tests that pivoting works as expected, using Example 4.7.7 from
// [Cottle 1992], p. 273.
TEST_F(UnrevisedLemkePrivateTests, LemkePivot) {
  typedef UnrevisedLemkeSolver<double>::LCPVariable LCPVariable;

  // Use the computed zero tolerance.
  double zero_tol = lcp_.ComputeZeroTolerance(M_);

  // Use the blocking index that Cottle provides us with.
  const int blocking_index = 0;

  auto blocking = lcp_.dep_variables_[blocking_index];
  int driving_index = blocking.index();
  std::swap(lcp_.dep_variables_[blocking_index],
            lcp_.indep_variables_[kArtificial]);

  // Case 1: Driving variable is from 'z'.
  // Compute the pivot and verify the result.
  VectorX<double> q_bar(3);
  VectorX<double> M_bar_col(3);
  ASSERT_TRUE(
      lcp_.LemkePivot(M_, q_, driving_index, zero_tol, &M_bar_col, &q_bar));
  VectorX<double> M_bar_col_expected(3);
  VectorX<double> q_bar_expected(3);
  M_bar_col_expected << 0, 2, -1;
  q_bar_expected << 3, 9, 2;
  EXPECT_TRUE(CompareMatrices(M_bar_col, M_bar_col_expected, epsilon,
                              MatrixCompareType::absolute));

  // Case 2: Driving variable is from 'w'. We use the second-to-last tableaux
  // from Example 4.4.7.
  lcp_.dep_variables_[0] = LCPVariable(true, 3);    // artificial variable
  lcp_.dep_variables_[1] = LCPVariable(false, 0);
  lcp_.dep_variables_[2] = LCPVariable(true, 2);
  lcp_.indep_variables_[0] = LCPVariable(false, 1);
  lcp_.indep_variables_[1] = LCPVariable(false, 2);
  lcp_.indep_variables_[2] = LCPVariable(true, 2);
  lcp_.indep_variables_[3] = LCPVariable(true, 0);
  driving_index = 0;
  ASSERT_TRUE(
      lcp_.LemkePivot(M_, q_, driving_index, zero_tol, &M_bar_col, &q_bar));
  M_bar_col_expected << 0, -1, -0.5;
  q_bar_expected << 1, 5, 3;
  EXPECT_TRUE(CompareMatrices(M_bar_col, M_bar_col_expected, epsilon,
                              MatrixCompareType::absolute));

  // Case 3: Pivoting in artificial variable (no M bar column passed in).
  // This is equivalent to the last tableaux of Example 4.4.7.
  lcp_.dep_variables_[0] = LCPVariable(true, 1);
  lcp_.dep_variables_[1] = LCPVariable(false, 0);
  lcp_.dep_variables_[2] = LCPVariable(true, 2);
  lcp_.indep_variables_[0] = LCPVariable(false, 1);
  lcp_.indep_variables_[1] = LCPVariable(false, 2);
  lcp_.indep_variables_[2] = LCPVariable(true, 3);  // artificial variable
  lcp_.indep_variables_[3] = LCPVariable(true, 0);
  driving_index = 2;
  ASSERT_TRUE(
      lcp_.LemkePivot(M_, q_, driving_index, zero_tol, nullptr, &q_bar));
  q_bar_expected << 0, 1, 3;
}

TEST_F(UnrevisedLemkePrivateTests, ConstructLemkeSolution) {
  typedef UnrevisedLemkeSolver<double>::LCPVariable LCPVariable;

  // Set the variables as expected in the last tableaux of Example 4.4.7.
  lcp_.dep_variables_[0] = LCPVariable(true, 1);
  lcp_.dep_variables_[1] = LCPVariable(false, 0);
  lcp_.dep_variables_[2] = LCPVariable(true, 2);
  lcp_.indep_variables_[0] = LCPVariable(false, 1);
  lcp_.indep_variables_[1] = LCPVariable(false, 2);
  lcp_.indep_variables_[2] = LCPVariable(true, 3);  // artificial variable
  lcp_.indep_variables_[3] = LCPVariable(true, 0);

  // Set the location of the artificial variable.
  int artificial_index_loc = 2;

  // Use the computed zero tolerance.
  double zero_tol = lcp_.ComputeZeroTolerance(M_);

  // Verify that the operation completes successfully.
  VectorX<double> z;
  ASSERT_TRUE(lcp_.ConstructLemkeSolution(
      M_, q_, artificial_index_loc, zero_tol, &z));

  // Verify that the solution is as expected.
  VectorX<double> z_expected(3);
  z_expected << 0, 1, 3;
  EXPECT_TRUE(CompareMatrices(z, z_expected, epsilon,
                              MatrixCompareType::absolute));
}

// Verifies that DetermineIndexSets() works as expected.
TEST_F(UnrevisedLemkePrivateTests, DetermineIndexSets) {
  typedef UnrevisedLemkeSolver<double>::LCPVariable LCPVariable;

  // Set indep_variables_ and dep_variables_ as in Equation (1) of [1].
  // Note: this equation must be kept up-to-date with equations in [1].
  lcp_.dep_variables_[0] = LCPVariable(true, 3);  // artificial variable.
  lcp_.dep_variables_[1] = LCPVariable(false, 1);
  lcp_.dep_variables_[2] = LCPVariable(true, 2);
  lcp_.indep_variables_[0] = LCPVariable(false, 0);
  lcp_.indep_variables_[1] = LCPVariable(false, 2);
  lcp_.indep_variables_[2] = LCPVariable(true, 1);
  lcp_.indep_variables_[3] = LCPVariable(true, 0);

  // Compute the index sets (uses indep_variables_ and dep_variables_).
  lcp_.DetermineIndexSets();

  // Verify that the sets have indices we expect (from [1]).
  ASSERT_EQ(lcp_.index_sets_.alpha.size(), 2);
  EXPECT_EQ(lcp_.index_sets_.alpha[0], 0);
  EXPECT_EQ(lcp_.index_sets_.alpha[1], 2);

  ASSERT_EQ(lcp_.index_sets_.alpha_prime.size(), 2);
  EXPECT_EQ(lcp_.index_sets_.alpha_prime[0], 0);
  EXPECT_EQ(lcp_.index_sets_.alpha_prime[1], 1);

  ASSERT_EQ(lcp_.index_sets_.beta.size(), 2);
  EXPECT_EQ(lcp_.index_sets_.beta[0], 2);
  EXPECT_EQ(lcp_.index_sets_.beta[1], 3);

  ASSERT_EQ(lcp_.index_sets_.beta_prime.size(), 2);
  EXPECT_EQ(lcp_.index_sets_.beta_prime[0], 2);
  EXPECT_EQ(lcp_.index_sets_.beta_prime[1], 0);

  EXPECT_EQ(lcp_.index_sets_.alpha_bar.size(), 1);
  EXPECT_EQ(lcp_.index_sets_.alpha_bar[0], 1);

  EXPECT_EQ(lcp_.index_sets_.alpha_bar_prime.size(), 1);
  EXPECT_EQ(lcp_.index_sets_.alpha_bar_prime[0], 1);

  EXPECT_EQ(lcp_.index_sets_.beta_bar.size(), 2);
  EXPECT_EQ(lcp_.index_sets_.beta_bar[0], 0);
  EXPECT_EQ(lcp_.index_sets_.beta_bar[1], 1);

  EXPECT_EQ(lcp_.index_sets_.beta_bar_prime.size(), 2);
  EXPECT_EQ(lcp_.index_sets_.beta_bar_prime[0], 3);
  EXPECT_EQ(lcp_.index_sets_.beta_bar_prime[1], 2);
}

// Verifies that finding the index of the complement of an independent variable
// works as expected.
TEST_F(UnrevisedLemkePrivateTests, FindComplementIndex) {
  // From the setup of the LCP solver designated by SetUp(), all z variables
  // (including the artificial one are independent). The query variable will
  // be w1, meaning that we expect the second variable (i.e., z1) to be
  // the complement.
  typedef UnrevisedLemkeSolver<double>::LCPVariable LCPVariable;
  LCPVariable query(false /* w */, 1);

  // We have to manually set the mapping from independent variables to their
  // indices, since the solver normally maintains this for us.
  for (int i = 0; i < static_cast<int>(lcp_.indep_variables_.size()); ++i)
    lcp_.indep_variables_indices_[lcp_.indep_variables_[i]] = i;

  // Since the indices of the LCP variables from SetUp()
  // correspond to their array indices, verification is straightforward.
  EXPECT_EQ(lcp_.FindComplementIndex(query), 1);
}

TEST_F(UnrevisedLemkePrivateTests, FindBlockingIndex) {
  // Use the computed zero tolerance.
  double zero_tol = lcp_.ComputeZeroTolerance(M_);

  // Ratios are taken from '1' column (the q vector) in the first tableaux from
  // Example 4.3.3. Note that this is the exact procedure used to find the first
  // blocking variable, which means we can check our answer against Cottle's.
  VectorX<double> col(3);
  col << -3, 6, 1;
  VectorX<double> ratios = col;

  // Index should be the first one.
  int blocking_index = -1;
  ASSERT_TRUE(lcp_.FindBlockingIndex(zero_tol, col, ratios, &blocking_index));
  EXPECT_EQ(blocking_index, 0);

  // Repeat the procedure using the second tableaux from Example 4.3.3. We
  // now compute the ratios manually using component-wise division of the column
  // marked '1' over the column marked 'z1'.
  col << 0, 2, -1;
  // NOTE: we replace 3.0 / 0 with infinity below to avoid divide by zero
  // warnings from the compiler.
  const double inf = std::numeric_limits<double>::infinity();
  ratios << inf, 9.0 / 2, 2.0 / -1.0;
  ASSERT_TRUE(lcp_.FindBlockingIndex(zero_tol, col, ratios, &blocking_index));
  EXPECT_EQ(blocking_index, 2);  // Blocking index must be the last entry.

  // Repeat the procedure, now using strictly positive column entries; no
  // blocking index should be possible.
  col << 0, 2, 1;
  ASSERT_FALSE(lcp_.FindBlockingIndex(zero_tol, col, ratios, &blocking_index));
  EXPECT_EQ(blocking_index, -1);  // Check that blocking index is invalid.
}

TEST_F(UnrevisedLemkePrivateTests, FindBlockingIndexCycling) {
  // Use the computed zero tolerance.
  double zero_tol = lcp_.ComputeZeroTolerance(M_);

  // We will have the column be the same as the ratios. This means that there
  // will be exactly two valid ratios, both identical.
  VectorX<double> col(3);
  col << -3, -3, 1;
  VectorX<double> ratios = col;

  // Index should be the first one.
  int blocking_index = -1;
  ASSERT_TRUE(lcp_.FindBlockingIndex(zero_tol, col, ratios, &blocking_index));
  EXPECT_EQ(blocking_index, 0);

  // Repeat the procedure again. Index should be the next one.
  ASSERT_TRUE(lcp_.FindBlockingIndex(zero_tol, col, ratios, &blocking_index));
  EXPECT_EQ(blocking_index, 1);

  // If we repeat one more time, there are no indices remaining.
  ASSERT_FALSE(lcp_.FindBlockingIndex(zero_tol, col, ratios, &blocking_index));
  EXPECT_EQ(blocking_index, -1);  // Check that blocking index is invalid.
}

}  // namespace solvers
}  // namespace drake
