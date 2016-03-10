
#include <iostream>
#include <memory>

#include "drake/solvers/MobyLCP.h"

namespace {

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

  Eigen::VectorXd fast_z;
  Drake::MobyLCPSolver l;
  l.setLoggingEnabled(true);
  bool result = l.lcp_fast(M, q, &fast_z);
  std::cout << "Result (lcp_fast): " << result << std::endl
            << fast_z << std::endl;

  Eigen::VectorXd lemke_z;
  result = l.lcp_lemke(M, q, &lemke_z);
  std::cout << "Result (lcp_lemke): " << result << std::endl
            << lemke_z << std::endl;
  assert(fast_z.isApprox(lemke_z));
  
  Eigen::SparseMatrix<double> M_sparse(9, 9);
  for (int i = 0; i < 9; i++) {
    M_sparse.insert(i, i) = i + 1;
  }
  lemke_z.setZero();
  result = l.lcp_lemke(M_sparse, q, &lemke_z);
  std::cout << "Result (lcp_lemke (sparse)): " << result << std::endl
            << lemke_z << std::endl;
  assert(fast_z.isApprox(lemke_z));

  // Mangle the input matrix so that some regularization occurs.
  M(0,8) = 10;
  M_sparse.coeffRef(0, 8) = 10;
  fast_z.setZero();
  result = l.lcp_fast_regularized(M, q, &fast_z);
  std::cout << "Result (fast regularized): " << result << std::endl
            << fast_z << std::endl;

  // TODO sammy this matrix doesn't appear to be mangled enough to
  // require regularization in the lemke case.
  lemke_z.setZero();
  result = l.lcp_lemke_regularized(M, q, &lemke_z);
  std::cout << "Result (lemke regularized): " << result << std::endl
            << lemke_z << std::endl;
  assert(fast_z.isApprox(lemke_z, 1e-6));

  lemke_z.setZero();
  result = l.lcp_lemke_regularized(M_sparse, q, &lemke_z);
  std::cout << "Result (lemke regularized (sparse)): " << result << std::endl
            << lemke_z << std::endl;
  assert(fast_z.isApprox(lemke_z, 1e-6));
}

void testEmpty() {
  Eigen::MatrixXd empty_M(0, 0);
  Eigen::VectorXd empty_q(0);
  Eigen::VectorXd z;
  Drake::MobyLCPSolver l;
  l.setLoggingEnabled(true);
  
  bool result = l.lcp_fast(empty_M, empty_q, &z);
  assert(result);
  assert(z.size() == 0);

  result = l.lcp_lemke(empty_M, empty_q, &z);
  assert(result);
  assert(z.size() == 0);

  Eigen::SparseMatrix<double> empty_sparse_M(0,0);
  result = l.lcp_lemke(empty_sparse_M, empty_q, &z);
  assert(result);
  assert(z.size() == 0);

  result = l.lcp_fast_regularized(empty_M, empty_q, &z);
  assert(result);
  assert(z.size() == 0);

  result = l.lcp_lemke_regularized(empty_M, empty_q, &z);
  assert(result);
  assert(z.size() == 0);

  result = l.lcp_lemke_regularized(empty_sparse_M, empty_q, &z);
  assert(result);
  assert(z.size() == 0);
}

}

int main(int argc, char* argv[]) {
  testTrivial();
  testEmpty();
  return 0;
}
