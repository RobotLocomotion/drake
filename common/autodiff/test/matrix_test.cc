#include <gtest/gtest.h>

#include "drake/common/autodiff.h"

namespace drake {
namespace autodiff {
namespace {

using Eigen::VectorXd;

// Provide some autodiff variables that don't expire at scope end for test
// cases.
class MatrixTest : public ::testing::Test {
 protected:
  AutoDiff x_{0.4, VectorXd::Ones(3)};
  AutoDiff y_{0.3, VectorXd::Ones(3)};
};

TEST_F(MatrixTest, MatrixProduct) {
  MatrixX<AutoDiff> A(2, 3);
  A << 0, x_, y_, y_, x_, 0;

  MatrixX<AutoDiff> B(3, 1);
  B << x_, y_, 0;

  MatrixX<AutoDiff> C = A * B;
}

}  // namespace
}  // namespace autodiff
}  // namespace drake
