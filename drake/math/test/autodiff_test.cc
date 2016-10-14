#include "drake/math/autodiff.h"

#include <Eigen/Dense>
#include <unsupported/Eigen/AutoDiff>

#include "gtest/gtest.h"

#include "drake/common/eigen_matrix_compare.h"
#include "drake/common/eigen_types.h"
#include "drake/math/autodiff_gradient.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

namespace drake {
namespace math {
namespace {

class AutodiffTest : public ::testing::Test {
 protected:
  typedef Eigen::AutoDiffScalar<VectorXd> Scalar;

  void SetUp() override {
    // The initial value of vec_ is [1.0, 10.0].
    vec_.resize(2);
    vec_[0].value() = 1.0;
    vec_[1].value() = 10.0;
    // We arbitrarily decide that the first element of the derivatives vector
    // corresponds to vec_[0], and the second to vec_[1].
    // The derivative of vec_[0] with respect to itself is initially 1, and
    // with respect to vec_1 is initially 0: [1.0, 0.0].
    vec_[0].derivatives() = Eigen::VectorXd::Unit(2, 0);
    // The derivative of vec_[1] with respect to itself is initially 1, and
    // with respect to vec_0 is initially 0: [0.0, 1.0].
    vec_[1].derivatives() = Eigen::VectorXd::Unit(2, 1);

    vec_ = DoMath(vec_);
  }

  // Computes a function in R^2 that has analytically easy partial
  // derivatives.
  static VectorX<Scalar> DoMath(const VectorX<Scalar>& vec) {
    VectorX<Scalar> output(2);
    // y1 = sin(v0) + v1
    output[1] = sin(vec[0]) + vec[1];
    // y0 = (sin(v0) + v1) * (cos(v0) / v1)
    //    = cos(v0) + (sin(v0) * cos(v0) / v1)
    output[0] = cos(vec[0]) / vec[1];
    output[0] *= output[1];
    return output;
  }

  VectorX<Scalar> vec_;
};

// Tests that ToValueMatrix extracts the values from the autodiff.
TEST_F(AutodiffTest, ToValueMatrix) {
  VectorXd values = autoDiffToValueMatrix(vec_);

  VectorXd expected(2);
  expected[1] = sin(1.0) + 10.0;
  expected[0] = (cos(1.0) / 10.0) * expected[1];
  EXPECT_TRUE(
      CompareMatrices(expected, values, 1e-10, MatrixCompareType::absolute))
      << values;
}

// Tests that ToGradientMatrix extracts the gradients from the autodiff.
TEST_F(AutodiffTest, ToGradientMatrix) {
  MatrixXd gradients = autoDiffToGradientMatrix(vec_);

  MatrixXd expected(2, 2);

  // By the product rule, the derivative of y0 with respect to v0 is
  // -sin(v0) + (1 / v1) * (cos(v0)^2 - sin(v0)^2)
  expected(0, 0) =
      -sin(1.0) + 0.1 * (cos(1.0) * cos(1.0) - sin(1.0) * sin(1.0));
  // The derivative of y0 with respect to v1 is sin(v0) * cos(v0) * -1 * v1^-2
  expected(0, 1) = sin(1.0) * cos(1.0) * -1.0 / (10.0 * 10.0);
  // The derivative of y1 with respect to v0 is cos(v0).
  expected(1, 0) = cos(1.0);
  // The derivative of y1 with respect to v1 is 1.
  expected(1, 1) = 1.0;

  EXPECT_TRUE(
      CompareMatrices(expected, gradients, 1e-10, MatrixCompareType::absolute))
      << gradients;
}

}  // namespace
}  // namespace math
}  // namespace drake
