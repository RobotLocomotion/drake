#pragma once

#include <cmath>
#include <iostream>

#include <Eigen/Dense>
#include <gtest/gtest.h>
#include "drake/solvers/fbstab/components/dense_data.h"
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

    y.axpy(x, a);

    for (int i = 0; i < n_; i++) {
      EXPECT_DOUBLE_EQ(y.z()(i), z_expected(i));
    }
    for (int i = 0; i < q_; i++) {
      EXPECT_DOUBLE_EQ(y.v()(i), v_expected(i));
      EXPECT_DOUBLE_EQ(y.y()(i), y_expected(i));
    }
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
