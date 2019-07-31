#pragma once

#include <iostream>
#include <vector>

#include <Eigen/Dense>
#include <gtest/gtest.h>

#include "drake/solvers/fbstab/components/mpc_data.h"
#include "drake/solvers/fbstab/components/mpc_variable.h"

namespace drake {
namespace solvers {
namespace fbstab {
namespace test {

using MatrixXd = Eigen::MatrixXd;
using VectorXd = Eigen::VectorXd;

/**
 * This class implements unit tests for the following classes:
 * MpcData
 * MpcVariable
 * MPCResidual
 * MPCFeasibiity
 * RicattiLinearSolver
 */
class MPCComponentUnitTests {
 public:
  /**
   * Sets up the optimal control problem
   * that will be used to test the component objects.
   *
   * The mathematical format can be found in
   * https://arxiv.org/pdf/1901.04046.pdf Eq. (29) or in mpc_data.h.
   */
  MPCComponentUnitTests() {
    int N = 2;  // Use a horizon length of 2 throughout.

    MatrixXd Q(2, 2);
    MatrixXd R(1, 1);
    MatrixXd S(1, 2);
    VectorXd q(2);
    VectorXd r(1);

    MatrixXd A(2, 2);
    MatrixXd B(2, 1);
    VectorXd c(2);

    MatrixXd E(6, 2);
    MatrixXd L(6, 1);
    VectorXd d(6);

    VectorXd x0(2);

    Q << 2, 0, 0, 1;
    S << 1, 0;
    R << 3;
    q << -2, 0;
    r << 0;

    A << 1, 1, 0, 1;
    B << 0, 1;
    c << 0, 0;

    E << -1, 0, 0, -1, 1, 0, 0, 1, 0, 0, 0, 0;
    L << 0, 0, 0, 0, -1, 1;
    d << 0, 0, -2, -2, -1, -1;

    x0 << 0, 0;

    // These are indexed from 0 to N.
    for (int i = 0; i < N + 1; i++) {
      Q_.push_back(Q);
      R_.push_back(R);
      S_.push_back(S);
      q_.push_back(q);
      r_.push_back(r);

      E_.push_back(E);
      L_.push_back(L);
      d_.push_back(d);
    }

    // These are indexed from 0 to N-1.
    for (int i = 0; i < N; i++) {
      A_.push_back(A);
      B_.push_back(B);
      c_.push_back(c);
    }

    x0_ = x0;
  }

  /**
   * Tests against hand calculations.
   */
  void GEMVH() {
    MpcData data(&Q_, &R_, &S_, &q_, &r_, &A_, &B_, &c_, &E_, &L_, &d_, &x0_);
    const int nz = data.nz_;

    VectorXd z(nz);
    for (int i = 0; i < nz; i++) {
      z(i) = i + 1;
    }

    VectorXd y(nz);

    data.gemvH(z, 1.0, 0.0, &y);

    VectorXd y_expected(nz);
    y_expected << 5, 2, 10, 14, 5, 22, 23, 8, 34;

    for (int i = 0; i < y.size(); i++) {
      ASSERT_EQ(y(i), y_expected(i));
    }
  }

  /**
   * Tests against hand calculations.
   */
  void GEMVA() {
    MpcData data(&Q_, &R_, &S_, &q_, &r_, &A_, &B_, &c_, &E_, &L_, &d_, &x0_);
    const int nv = data.nv_;
    const int nz = data.nz_;

    VectorXd z(nz);
    for (int i = 0; i < nz; i++) {
      z(i) = i + 1;
    }

    VectorXd y(nv);
    data.gemvA(z, 1.0, 0.0, &y);

    VectorXd y_expected(nv);
    y_expected << -1, -2, 1, 2, -3, 3, -4, -5, 4, 5, -6, 6, -7, -8, 7, 8, -9, 9;

    for (int i = 0; i < y.size(); i++) {
      ASSERT_EQ(y(i), y_expected(i));
    }
  }

  /**
   * Tests against hand calculations.
   */
  void GEMVG() {
    MpcData data(&Q_, &R_, &S_, &q_, &r_, &A_, &B_, &c_, &E_, &L_, &d_, &x0_);
    const int nl = data.nl_;
    const int nz = data.nz_;

    VectorXd z(nz);
    for (int i = 0; i < nz; i++) {
      z(i) = i + 1;
    }

    VectorXd y(nl);
    data.gemvG(z, 1.0, 0.0, &y);

    VectorXd y_expected(nl);
    y_expected << -1, -2, -1, 0, 2, 3;

    for (int i = 0; i < y.size(); i++) {
      ASSERT_EQ(y(i), y_expected(i));
    }
  }

  /**
   * Tests against hand calculations.
   */
  void GEMVGT() {
    MpcData data(&Q_, &R_, &S_, &q_, &r_, &A_, &B_, &c_, &E_, &L_, &d_, &x0_);
    const int nl = data.nl_;
    const int nz = data.nz_;

    VectorXd l(nl);
    for (int i = 0; i < nl; i++) {
      l(i) = i + 1;
    }

    VectorXd y(nz);
    data.gemvGT(l, 1.0, 0.0, &y);

    VectorXd y_expected(nz);
    y_expected << 2, 5, 4, 2, 7, 6, -5, -6, 0;

    for (int i = 0; i < y.size(); i++) {
      ASSERT_EQ(y(i), y_expected(i));
    }
  }

  /**
   * Tests against hand calculations.
   */
  void GEMVAT() {
    MpcData data(&Q_, &R_, &S_, &q_, &r_, &A_, &B_, &c_, &E_, &L_, &d_, &x0_);
    const int nv = data.nv_;
    const int nz = data.nz_;

    VectorXd v(nv);
    for (int i = 0; i < nv; i++) {
      v(i) = i + 1;
    }

    VectorXd y(nz);
    data.gemvAT(v, 1.0, 0.0, &y);

    VectorXd y_expected(nz);
    y_expected << 2, 2, 1, 2, 2, 1, 2, 2, 1;

    for (int i = 0; i < y.size(); i++) {
      ASSERT_EQ(y(i), y_expected(i));
    }
  }

  /**
   * Tests against hand calculations.
   */
  void AXPYF() {
    MpcData data(&Q_, &R_, &S_, &q_, &r_, &A_, &B_, &c_, &E_, &L_, &d_, &x0_);
    const int nz = data.nz_;

    VectorXd y(nz);
    y << 5, 2, 10, 14, 5, 22, 23, 8, 34;
    data.axpyf(2.0, &y);

    VectorXd y_expected(nz);
    y_expected << 1, 2, 10, 10, 5, 22, 19, 8, 34;

    for (int i = 0; i < y.size(); i++) {
      ASSERT_EQ(y(i), y_expected(i));
    }
  }

  /**
   * Tests against hand calculations.
   */
  void AXPYH() {
    MpcData data(&Q_, &R_, &S_, &q_, &r_, &A_, &B_, &c_, &E_, &L_, &d_, &x0_);
    const int nl = data.nl_;

    VectorXd y(nl);
    y << -1, -2, -1, 0, 2, 3;
    data.axpyh(2.0, &y);

    VectorXd y_expected(nl);
    y_expected << -1, -2, -1, 0, 2, 3;

    for (int i = 0; i < y.size(); i++) {
      ASSERT_EQ(y(i), y_expected(i));
    }
  }

  /**
   * Tests against hand calculations.
   */
  void AXPYB() {
    MpcData data(&Q_, &R_, &S_, &q_, &r_, &A_, &B_, &c_, &E_, &L_, &d_, &x0_);
    const int nv = data.nv_;

    VectorXd y(nv);
    y << -1, -2, 1, 2, -3, 3, -4, -5, 4, 5, -6, 6, -7, -8, 7, 8, -9, 9;
    data.axpyb(2.0, &y);

    VectorXd y_expected(nv);
    y_expected << -1, -2, 5, 6, -1, 5, -4, -5, 8, 9, -4, 8, -7, -8, 11, 12, -7,
        11;

    for (int i = 0; i < y.size(); i++) {
      ASSERT_EQ(y(i), y_expected(i));
    }
  }

  /**
   * Tests against hand calculations.
   */
  void Variable() {
    MpcData data(&Q_, &R_, &S_, &q_, &r_, &A_, &B_, &c_, &E_, &L_, &d_, &x0_);

    MpcVariable x(data.N_, data.nx_, data.nu_, data.nc_);
    MpcVariable y(data.N_, data.nx_, data.nu_, data.nc_);
    x.LinkData(&data);
    y.LinkData(&data);

    x.Fill(1.0);
    y.Fill(1.0);

    x.axpy(-2.0, y);

    VectorXd z_expected(data.nz_);
    VectorXd l_expected(data.nl_);
    VectorXd v_expected(data.nv_);
    VectorXd y_expected(data.nv_);

    z_expected << -1, -1, -1, -1, -1, -1, -1, -1, -1;
    l_expected << -1, -1, -1, -1, -1, -1;
    v_expected << -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
        -1, -1, -1;
    y_expected << -1, -1, 3, 3, 0, 2, -1, -1, 3, 3, 0, 2, -1, -1, 3, 3, 0, 2;

    for (int i = 0; i < z_expected.size(); i++) {
      ASSERT_EQ(x.z()(i), z_expected(i));
    }
    for (int i = 0; i < l_expected.size(); i++) {
      ASSERT_EQ(x.l()(i), l_expected(i));
    }
    for (int i = 0; i < y_expected.size(); i++) {
      ASSERT_EQ(x.y()(i), y_expected(i));
      ASSERT_EQ(x.v()(i), v_expected(i));
    }
  }

 private:
  std::vector<Eigen::MatrixXd> Q_;
  std::vector<Eigen::MatrixXd> R_;
  std::vector<Eigen::MatrixXd> S_;
  std::vector<Eigen::VectorXd> q_;
  std::vector<Eigen::VectorXd> r_;

  std::vector<Eigen::MatrixXd> A_;
  std::vector<Eigen::MatrixXd> B_;
  std::vector<Eigen::VectorXd> c_;

  std::vector<Eigen::MatrixXd> E_;
  std::vector<Eigen::MatrixXd> L_;
  std::vector<Eigen::VectorXd> d_;

  Eigen::VectorXd x0_;
};

}  // namespace test
}  // namespace fbstab
}  // namespace solvers
}  // namespace drake
