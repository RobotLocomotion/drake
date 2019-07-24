#include "drake/solvers/fbstab/fbstab_dense.h"

#include <cmath>
#include <Eigen/Dense>
#include <gtest/gtest.h>

namespace drake {
namespace solvers {
namespace fbstab {
namespace test {

using MatrixXd = Eigen::MatrixXd;
using VectorXd = Eigen::VectorXd;

/**
 * @file Unit tests for FBstabDense
 * which is designed to solve QPs of the form:
 *
 * min  0.5 z'Hz + f'z
 * s.t. Az <= b
 *
 */

/**
 * Tests FBstab with
 *
 * H = [3 1]  f = [10]
 *     [1 1]      [5 ]
 *
 * A = [-1 0] b = [0]
 *     [0  1]     [0]
 *
 * This QP can be solved analytically
 * and has the unique primal(z) - dual(v) solution
 * z = [0 -5],  v = [5 0]
 */
GTEST_TEST(FBstabDense, FeasibleQP) {
  MatrixXd H(2, 2);
  MatrixXd A(2, 2);
  VectorXd f(2);
  VectorXd b(2);

  H << 3, 1, 1, 1;
  f << 10, 5;
  A << -1, 0, 0, 1;
  b << 0, 0;

  int n = f.size();
  int q = b.size();

  DenseQPData data;
  data.H = &H;
  data.f = &f;
  data.A = &A;
  data.b = &b;

  VectorXd z0 = Eigen::VectorXd::Zero(n);
  VectorXd v0 = Eigen::VectorXd::Zero(q);
  VectorXd y0 = Eigen::VectorXd::Zero(q);

  DenseQPVariable x0;
  x0.z = &z0;
  x0.v = &v0;
  x0.y = &y0;

  FBstabDense solver(n, q);
  solver.UpdateOption("abs_tol", 1e-8);
  solver.SetDisplayLevel(FBstabAlgoDense::OFF);
  SolverOut out = solver.Solve(data, x0);

  ASSERT_EQ(out.eflag, SUCCESS);

  VectorXd zopt(2);
  VectorXd vopt(2);
  zopt << 0, -5;
  vopt << 5, 0;
  for (int i = 0; i < n; i++) {
    EXPECT_NEAR(z0(i), zopt(i), 1e-8);
  }

  for (int i = 0; i < q; i++) {
    EXPECT_NEAR(v0(i), vopt(i), 1e-8);
  }
}

/**
 * Tests FBstab with
 *
 * H = [1 0]  f = [1]
 *     [0 0]      [0]
 *
 * A = [0  0] b = [0 ]
 *     [1  0]     [3 ]
 *     [0  1]     [3 ]
 *     [-1 0]     [-1]
 *     [0 -1]     [-1]
 *
 * This QP is degenerate with a primal solution set
 * [1] x [1,3]
 */
GTEST_TEST(FBstabDense, DegenerateQP) {
  MatrixXd H(2, 2);
  MatrixXd A(5, 2);
  VectorXd f(2);
  VectorXd b(5);

  H << 1, 0, 0, 0;
  f << 1, 0;

  A << 0, 0, 1, 0, 0, 1, -1, 0, 0, -1;

  b << 0, 3, 3, -1, -1;

  int n = f.size();
  int q = b.size();

  DenseQPData data;
  data.H = &H;
  data.f = &f;
  data.A = &A;
  data.b = &b;

  VectorXd z0 = Eigen::VectorXd::Zero(n);
  VectorXd v0 = Eigen::VectorXd::Zero(q);
  VectorXd y0 = Eigen::VectorXd::Zero(q);

  DenseQPVariable x0;
  x0.z = &z0;
  x0.v = &v0;
  x0.y = &y0;

  FBstabDense solver(n, q);
  solver.UpdateOption("abs_tol", 1e-8);
  solver.SetDisplayLevel(FBstabAlgoDense::OFF);
  SolverOut out = solver.Solve(data, x0);

  ASSERT_EQ(out.eflag, SUCCESS);
  EXPECT_NEAR(z0(0), 1, 1e-8);
  EXPECT_TRUE((z0(1) >= 1) && (z0(1) <= 3));
}

/**
 * Tests FBstab with
 *
 * H = [1 0]  f = [1 ]
 *     [0 0]      [-1]
 *
 * A = [1  1] b = [0 ]
 *     [1  0]     [3 ]
 *     [0  1]     [3 ]
 *     [-1 0]     [-1]
 *     [0 -1]     [-1]
 *
 * This QP is infeasible, i.e.,
 * there is no z satisfying Az <= b
 */
GTEST_TEST(FBstabDense, InfeasibleQP) {
  MatrixXd H(2, 2);
  MatrixXd A(5, 2);
  VectorXd f(2);
  VectorXd b(5);

  H << 1, 0, 0, 0;
  f << 1, -1;

  A << 1, 1, 1, 0, 0, 1, -1, 0, 0, -1;

  b << 0, 3, 3, -1, -1;

  int n = f.size();
  int q = b.size();

  DenseQPData data;
  data.H = &H;
  data.f = &f;
  data.A = &A;
  data.b = &b;

  VectorXd z0 = Eigen::VectorXd::Zero(n);
  VectorXd v0 = Eigen::VectorXd::Zero(q);
  VectorXd y0 = Eigen::VectorXd::Zero(q);

  DenseQPVariable x0;
  x0.z = &z0;
  x0.v = &v0;
  x0.y = &y0;

  FBstabDense solver(n, q);
  solver.UpdateOption("abs_tol", 1e-8);
  solver.SetDisplayLevel(FBstabAlgoDense::OFF);
  SolverOut out = solver.Solve(data, x0);

  ASSERT_EQ(out.eflag, PRIMAL_INFEASIBLE);
}

/**
 * Tests FBstab with
 *
 * H = [1 0]  f = [1 ]
 *     [0 0]      [-1]
 *
 * A = [0  0] b = [0 ]
 *     [1  0]     [3 ]
 *     [-1 0]     [-1]
 *     [0 -1]     [-1]
 *
 * This QP is unbounded below, i.e.,
 * its optimal value is -infinity
 */
GTEST_TEST(FBstabDense, UnboundedQP) {
  MatrixXd H(2, 2);
  MatrixXd A(4, 2);
  VectorXd f(2);
  VectorXd b(4);

  H << 1, 0, 0, 0;
  f << 1, -1;

  A << 0, 0, 1, 0, -1, 0, 0, -1;

  b << 0, 3, -1, -1;

  int n = f.size();
  int q = b.size();

  DenseQPData data;
  data.H = &H;
  data.f = &f;
  data.A = &A;
  data.b = &b;

  VectorXd z0 = Eigen::VectorXd::Zero(n);
  VectorXd v0 = Eigen::VectorXd::Zero(q);
  VectorXd y0 = Eigen::VectorXd::Zero(q);

  DenseQPVariable x0;
  x0.z = &z0;
  x0.v = &v0;
  x0.y = &y0;

  FBstabDense solver(n, q);
  solver.UpdateOption("abs_tol", 1e-8);
  solver.SetDisplayLevel(FBstabAlgoDense::OFF);
  SolverOut out = solver.Solve(data, x0);

  ASSERT_EQ(out.eflag, DUAL_INFEASIBLE);
}

}  // namespace test
}  // namespace fbstab
}  // namespace solvers
}  // namespace drake
