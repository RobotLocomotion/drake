
#include <iostream>
#include <memory>
#include <vector>

#include "drake/solvers/MobyLCP.h"
#include "drake/util/testUtil.h"

namespace {
const double epsilon = 1e-6;

template <typename Derived>
Eigen::SparseMatrix<double> makeSparseMatrix(
    const Eigen::MatrixBase<Derived>& M) {
  typedef Eigen::Triplet<double> Triplet;
  std::vector<Triplet> triplet_list;
  for (int i = 0; i < M.rows(); i++) {
    for (int j = 0; j < M.cols(); j++) {
      if (M(i, j)) {
        triplet_list.push_back(Triplet(i, j, M(i, j)));
      }
    }
  }
  Eigen::SparseMatrix<double> out(M.rows(), M.cols());
  out.setFromTriplets(triplet_list.begin(), triplet_list.end());
  return out;
}

/// Run all non-regularized solvers.  If @p expected_z is an empty
/// vector, outputs will only be compared against each other.
template <typename Derived>
void runBasicLCP(const Eigen::MatrixBase<Derived>& M, const Eigen::VectorXd& q,
                 const Eigen::VectorXd& expected_z_in, bool expect_fast_pass) {
  Drake::MobyLCPSolver l;
  l.setLoggingEnabled(true);

  Eigen::VectorXd expected_z = expected_z_in;

  Eigen::VectorXd fast_z;
  bool result = l.lcp_fast(M, q, &fast_z);
  std::cout << "Result (lcp_fast): " << result << std::endl
            << fast_z << std::endl;
  if (expected_z.size() == 0)  {
    if (!expect_fast_pass) {
      throw std::runtime_error(
          "Expected Z not provided and expect_fast_pass unset.");
    }
    expected_z = fast_z;
  } else if (expect_fast_pass) {
    valuecheckMatrix(fast_z, expected_z, epsilon);
  }

  Eigen::VectorXd lemke_z;
  result = l.lcp_lemke(M, q, &lemke_z);
  std::cout << "Result (lcp_lemke): " << result << std::endl
            << lemke_z << std::endl;
  valuecheckMatrix(lemke_z, expected_z, epsilon);
  
  Eigen::SparseMatrix<double> M_sparse = makeSparseMatrix(M);
  lemke_z.setZero();
  result = l.lcp_lemke(M_sparse, q, &lemke_z);
  std::cout << "Result (lcp_lemke (sparse)): " << result << std::endl
            << lemke_z << std::endl;
  valuecheckMatrix(lemke_z, expected_z, epsilon);
}

/// Run all regularized solvers.  If @p expected_z is an empty
/// vector, outputs will only be compared against each other.
template <typename Derived>
void runRegularizedLCP(const Eigen::MatrixBase<Derived>& M, const Eigen::VectorXd& q,
                       const Eigen::VectorXd& expected_z_in) {
  Drake::MobyLCPSolver l;
  l.setLoggingEnabled(true);

  Eigen::VectorXd expected_z = expected_z_in;

  Eigen::VectorXd fast_z;
  bool result = l.lcp_fast_regularized(M, q, &fast_z);
  std::cout << "Result (lcp_fast): " << result << std::endl
            << fast_z << std::endl;
  if (expected_z.size() == 0)  {
    expected_z = fast_z;
  } else {
    valuecheckMatrix(fast_z, expected_z, epsilon);
  }
  
  Eigen::VectorXd lemke_z;
  result = l.lcp_lemke_regularized(M, q, &lemke_z);
  std::cout << "Result (lcp_lemke): " << result << std::endl
            << lemke_z << std::endl;
  valuecheckMatrix(lemke_z, expected_z, epsilon);
  
  Eigen::SparseMatrix<double> M_sparse = makeSparseMatrix(M);
  lemke_z.setZero();
  result = l.lcp_lemke_regularized(M_sparse, q, &lemke_z);
  std::cout << "Result (lcp_lemke (sparse)): " << result << std::endl
            << lemke_z << std::endl;
  valuecheckMatrix(lemke_z, expected_z, epsilon);
}
  
void testTrivial() {
  Eigen::Matrix<double, 9, 9> M;
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

  Eigen::Matrix<double, 9, 1> q;
  q << -1, -1, -1, -1, -1, -1, -1, -1, -1;
  
  Eigen::VectorXd empty_z;
  runBasicLCP(M, q, empty_z, true);
    
  // Mangle the input matrix so that some regularization occurs.
  M(0,8) = 10;
  runRegularizedLCP(M, q, empty_z);
}

void testProblem1() {
  // Problem from example 10.2.1 in "Handbook of Test Problems in
  // Local and Global Optimization".
  Eigen::Matrix<double, 16, 16> M;
  M.setIdentity();
  for (int i = 0; i < M.rows() - 1; i++) {
    for (int j = i + 1; j < M.cols(); j++) {
      M(i,j) = 2;
    }
  }
  
  Eigen::Matrix<double, 1, 16> q;
  q.fill(-1);
  
  Eigen::Matrix<double, 1, 16> z;
  z.setZero();
  z(15) = 1;
  runBasicLCP(M, q, z, false);
}

void testProblem2() {
  // Problem from example 10.2.2 in "Handbook of Test Problems in
  // Local and Global Optimization".
  Eigen::Matrix<double, 2, 2> M;
  M.fill(1);

  Eigen::Matrix<double, 1, 2> q;
  q.fill(-1);
  
  // This problem also has valid solutions (0, 1) and (0.5, 0.5).
  Eigen::Matrix<double, 1, 2> z;
  z << 1, 0;

  runBasicLCP(M, q, z, true);
}

void testProblem3() {
  // Problem from example 10.2.3 in "Handbook of Test Problems in
  // Local and Global Optimization".
  Eigen::Matrix<double, 3, 3> M;
  M <<
      0, -1,  2,
      2,  0, -2,
      -1, 1,  0;
  Eigen::Matrix<double, 1, 3> q;
  q << -3, 6, -1;
  
  Eigen::Matrix<double, 1, 3> z;
  z << 0, 1, 3;

  runBasicLCP(M, q, z, false);
}

void testProblem4() {
  // Problem from example 10.2.4 in "Handbook of Test Problems in
  // Local and Global Optimization".
  Eigen::Matrix<double, 4, 4> M;
  M <<
      0,  0,  10,  20,
      0,  0,  30,  15,
      10, 20,  0,   0,
      30, 15,  0,   0;

  Eigen::Matrix<double, 1, 4> q;
  q.fill(-1);
  
  // This solution is the third in the book, which it explicitly
  // states cannot be found using the Lemke-Howson algorithm.
  Eigen::VectorXd z(4);
  z << 1./90., 2./45., 1./90., 2./45.;

  Drake::MobyLCPSolver l;
  l.setLoggingEnabled(true);

  Eigen::VectorXd fast_z(4);
  bool result = l.lcp_fast(M, q, &fast_z);
  std::cout << "Result (lcp_fast): " << result << std::endl
            << fast_z << std::endl;
  valuecheckMatrix(fast_z, z, epsilon);

  // TODO sammy the Lemke solvers find no solution at all, however.
  fast_z.setZero();
  result = l.lcp_lemke(M, q, &fast_z);
  if (result) {
    throw std::runtime_error("Did not expect solver to provide result.");
  }

  Eigen::SparseMatrix<double> M_sparse = makeSparseMatrix(M);
  result = l.lcp_lemke(M_sparse, q, &fast_z);
  if (result) {
    throw std::runtime_error("Did not expect solver to provide result.");
  }
}

void assertEmpty(const Eigen::VectorXd& z) {
  if (z.size() != 0) {
    throw std::runtime_error("Expected empty vector.");
  }
}

void testEmpty() {
  Eigen::MatrixXd empty_M(0, 0);
  Eigen::VectorXd empty_q(0);
  Eigen::VectorXd z;
  Drake::MobyLCPSolver l;
  l.setLoggingEnabled(true);
  
  bool result = l.lcp_fast(empty_M, empty_q, &z);
  assertEmpty(z);

  result = l.lcp_lemke(empty_M, empty_q, &z);
  assertEmpty(z);

  Eigen::SparseMatrix<double> empty_sparse_M(0,0);
  result = l.lcp_lemke(empty_sparse_M, empty_q, &z);
  assertEmpty(z);

  result = l.lcp_fast_regularized(empty_M, empty_q, &z);
  assertEmpty(z);

  result = l.lcp_lemke_regularized(empty_M, empty_q, &z);
  assertEmpty(z);

  result = l.lcp_lemke_regularized(empty_sparse_M, empty_q, &z);
  assertEmpty(z);
}

}

int main(int argc, char* argv[]) {
  testTrivial();
  testProblem1();
  testProblem2();
  testProblem3();
  testProblem4();
  testEmpty();
  return 0;
}
