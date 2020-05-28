#pragma once

#include <cmath>
#include <iostream>

#include <Eigen/Dense>
#include <gtest/gtest.h>

#include "drake/solvers/fbstab/components/dense_data.h"
#include "drake/solvers/fbstab/components/dense_feasibility.h"
#include "drake/solvers/fbstab/components/dense_linear_solver.h"
#include "drake/solvers/fbstab/components/dense_residual.h"
#include "drake/solvers/fbstab/components/dense_variable.h"

namespace drake {
namespace solvers {
namespace fbstab {
namespace test {

using MatrixXd = Eigen::MatrixXd;
using VectorXd = Eigen::VectorXd;

/**
 * This class implements unit tests for the following classes
 * DenseData
 * DenseVariable
 * DenseResidual
 * DenseFeasibiity
 * DenseLinearSolver
 */
class DenseComponentUnitTests {
 public:
  DenseComponentUnitTests() {
    H_.resize(2, 2);
    A_.resize(2, 2);
    f_.resize(2);
    b_.resize(2);
    n_ = f_.size();
    q_ = b_.size();

    H_ << 3, 1, 1, 1;
    A_ << -1, 0, 0, 1;
    f_ << 1, 6;
    b_ << 0, -1;
  }

  /**
   * This methods implements a unit test for the variable object.
   * It checks the computation of the constraint margin y = b - A*z against
   * Eigen and operations of the form y <- a*x + y
   * where x and y are DenseVariables.
   */
  void DenseVariableTests() {
    DenseData data(&H_, &f_, &A_, &b_);

    DenseVariable x(n_, q_);
    x.LinkData(&data);
    x.Fill(1.0);
    x.InitializeConstraintMargin();

    DenseVariable y(n_, q_);
    y.LinkData(&data);
    y.Fill(-1.0);
    y.InitializeConstraintMargin();

    // Test computation of the constraint margin
    // x.y =  b - A*x.z against hand calculations
    VectorXd margin_expected = b_ - A_ * x.z();
    for (int i = 0; i < q_; i++) {
      EXPECT_DOUBLE_EQ(x.y()(i), margin_expected(i));
    }

    // Test computation of the expression
    // y <- a*x + y
    double a = 0.35;

    VectorXd z_expected;
    VectorXd v_expected;
    VectorXd y_expected;

    z_expected = a * x.z() + y.z();
    v_expected = a * x.v() + y.v();
    y_expected = b_ - A_ * z_expected;

    y.axpy(a, x);

    for (int i = 0; i < n_; i++) {
      EXPECT_DOUBLE_EQ(y.z()(i), z_expected(i));
    }
    for (int i = 0; i < q_; i++) {
      EXPECT_DOUBLE_EQ(y.v()(i), v_expected(i));
      EXPECT_DOUBLE_EQ(y.y()(i), y_expected(i));
    }
  }

  // Compute the Inner Residual and compare against
  // hand calculations.
  // rz = H*x.z + f + A'*x.v + sigma*(x.z - y.z)
  // rv = phi(x.y + sigma*(x.v - y.v), x.v)
  // where phi(a,b) = 0.95* (a+b - sqrt(a^2 + b^2)) + 0.5*(max(0,a)*max(0,b))
  // is applied element-wise.
  void InnerResidualCalculation() {
    DenseData data(&H_, &f_, &A_, &b_);

    DenseVariable x(n_, q_);
    x.LinkData(&data);
    x.z() << 1, 5;
    x.v() << 0.4, 2;
    x.InitializeConstraintMargin();

    DenseVariable y(n_, q_);
    y.LinkData(&data);
    y.z() << -5, 6;
    y.v() << -9, 1;
    y.InitializeConstraintMargin();

    DenseResidual r(n_, q_);

    double sigma = 0.5;

    r.InnerResidual(x, y, sigma);
    VectorXd rz_expected(n_);
    rz_expected << 11.6, 13.5;
    VectorXd rv_expected(q_);
    rv_expected << 0.480683041678573, -8.88473245759182;

    for (int i = 0; i < n_; i++) {
      EXPECT_NEAR(r.z()(i), rz_expected(i), 1e-14);
    }
    for (int i = 0; i < q_; i++) {
      EXPECT_NEAR(r.v()(i), rv_expected(i), 1e-14);
    }
  }

  // Compute the natural residual and compare against hand
  // calculations.
  // rz = H*x.z + f + A'*x.v
  // rv = min(x.y,x.v)
  void NaturalResidualCalculation() {
    DenseData data(&H_, &f_, &A_, &b_);

    DenseVariable x(n_, q_);
    x.LinkData(&data);
    x.z() << 1, 5;
    x.v() << 0.4, 2;
    x.InitializeConstraintMargin();

    DenseResidual r(n_, q_);

    r.NaturalResidual(x);
    VectorXd rz_expected(n_);
    VectorXd rv_expected(q_);
    rz_expected << 8.6, 14.0;
    rv_expected << 0.4, -6;

    for (int i = 0; i < n_; i++) {
      EXPECT_NEAR(r.z()(i), rz_expected(i), 1e-14);
    }
    for (int i = 0; i < q_; i++) {
      EXPECT_NEAR(r.v()(i), rv_expected(i), 1e-14);
    }
  }

  // Verifies that the outputs of the Solve and
  // Factor methods do indeed solve the system
  //
  //      [ Hs   A']  dz  =  rz
  //      [-C*A  D ]  dv     rv
  //
  // where Hs = H + sigma*I, and
  // C = diag(gamma), D = diag(mus)
  // are diagonal weighting matrices computed from the derivatives
  // of the Penalized Fischer-Burmeister Function.
  void LinearSolverResidual() {
    DenseData data(&H_, &f_, &A_, &b_);

    DenseVariable x(n_, q_);
    x.LinkData(&data);
    x.z() << 1, 5;
    x.v() << 0.4, 2;
    x.InitializeConstraintMargin();

    DenseVariable y(n_, q_);
    y.LinkData(&data);
    y.z() << -5, 6;
    y.v() << -9, 1;
    y.InitializeConstraintMargin();

    DenseResidual r(n_, q_);
    r.Fill(1.0);

    DenseLinearSolver solver(n_, q_);

    double sigma = 0.5;
    solver.Initialize(x, y, sigma);
    solver.Solve(r, &y);

    // Construct the linear system
    // using the same diagonal matrices as were used in the solver.
    MatrixXd Hs = H_ + sigma * Eigen::MatrixXd::Identity(n_, n_);
    MatrixXd C = solver.gamma_.asDiagonal();
    MatrixXd D = solver.mus_.asDiagonal();

    MatrixXd K(n_ + q_, n_ + q_);
    K << Hs, A_.transpose(), -C * A_, D;

    VectorXd dx(n_ + q_);
    dx << y.z(), y.v();

    VectorXd rhs(n_ + q_);
    rhs << r.z(), r.v();

    VectorXd residual = K * dx - rhs;
    EXPECT_NEAR(residual.norm(), 0, 1e-12);
  }

  /**
   * This test checks that the infeasibility checker class correctly
   * identifies certificates of primal infeasibility.
   * The QP used in this example has no solution
   * and the vector [1 0 0 1 1] is a certificate of
   * infeasibility, i.e., it separates Range(A) and b.
   * The example is from https://arxiv.org/pdf/1901.04046.pdf
   */
  void PrimalInfeasibilityDetection() {
    MatrixXd H(2, 2);
    MatrixXd A(5, 2);
    VectorXd f(2);
    VectorXd b(5);

    H << 1, 0, 0, 0;

    A << 1, 1, 1, 0, 0, 1, -1, 0, 0, -1;

    f << 1, -1;
    b << 0, 3, 3, -1, -1;

    int n = f.size();
    int q = b.size();

    DenseData data(&H, &f, &A, &b);

    DenseVariable dx(n, q);
    dx.LinkData(&data);

    // the vector v = [1 0 0 1 1] is a certificate of primal infeasibility
    dx.Fill(0);
    dx.v() << 1, 0, 0, 1, 1;
    dx.InitializeConstraintMargin();

    DenseFeasibility feas(n, q);
    feas.ComputeFeasibility(dx, 1e-8);

    ASSERT_TRUE(feas.IsDualFeasible());
    ASSERT_FALSE(feas.IsPrimalFeasible());
  }

  /**
   * This test checks that the infeasibility checker class correctly
   * identifies certificates of dual infeasibility.
   * The QP used in this example has a direction of infinite descent
   * given by [0 1].
   * The example is taken from https://arxiv.org/pdf/1901.04046.pdf
   */
  void DualInfeasibilityDetection() {
    MatrixXd H(2, 2);
    MatrixXd A(4, 2);
    VectorXd f(2);
    VectorXd b(4);

    H << 1, 0, 0, 0;

    A << 0, 0, 1, 0, -1, 0, 0, -1;

    f << 1, -1;
    b << 0, 3, -1, -1;

    int n = f.size();
    int q = b.size();

    DenseData data(&H, &f, &A, &b);

    DenseVariable dx(n, q);
    dx.LinkData(&data);

    // The direction z = [0 1] is a direction of unbounded descent
    dx.Fill(0);
    dx.z() << 0, 1;

    DenseFeasibility feas(n, q);
    feas.ComputeFeasibility(dx, 1e-8);

    ASSERT_FALSE(feas.IsDualFeasible());
    ASSERT_TRUE(feas.IsPrimalFeasible());
  }

 private:
  MatrixXd H_;
  MatrixXd A_;
  VectorXd f_;
  VectorXd b_;

  int n_ = 0;
  int q_ = 0;
};

}  // namespace test
}  // namespace fbstab
}  // namespace solvers
}  // namespace drake
