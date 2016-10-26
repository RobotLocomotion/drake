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

class AutodiffMatrixConversionTest : public ::testing::Test {
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
TEST_F(AutodiffMatrixConversionTest, ToValueMatrix) {
  VectorXd values = autoDiffToValueMatrix(vec_);

  VectorXd expected(2);
  expected[1] = sin(1.0) + 10.0;
  expected[0] = (cos(1.0) / 10.0) * expected[1];
  EXPECT_TRUE(
      CompareMatrices(expected, values, 1e-10, MatrixCompareType::absolute))
      << values;
}

// Tests that ToGradientMatrix extracts the gradients from the autodiff.
TEST_F(AutodiffMatrixConversionTest, ToGradientMatrix) {
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

// Tests that pow(AutoDiffScalar, AutoDiffScalar) applies the chain rule.
GTEST_TEST(AutodiffTest, Pow) {
  Eigen::AutoDiffScalar<Eigen::Vector2d> x;
  x.value() = 1.1;
  x.derivatives() = Eigen::VectorXd::Unit(2, 0);
  Eigen::AutoDiffScalar<Eigen::Vector2d> y;
  y.value() = 2.5;
  y.derivatives() = Eigen::VectorXd::Unit(2 ,1);

  x = x * (y + 2);
  EXPECT_DOUBLE_EQ(4.95, x.value());

  // The derivative of x with respect to its original value is y + 2 = 4.5.
  EXPECT_DOUBLE_EQ(4.5, x.derivatives()[0]);
  // The derivative of x with respect to y is x = 1.1.
  EXPECT_DOUBLE_EQ(1.1, x.derivatives()[1]);

  auto z = pow(x, y);
  // z is x^y = 4.95^2.5 ~= 54.51.
  EXPECT_DOUBLE_EQ(std::pow(4.95, 2.5), z.value());
  // δz/δx is y*x^(y-1) = 2.5 * 4.95^1.5 ~= 27.53.
  const double dzdx = 2.5 * std::pow(4.95, 1.5);
  // δz/δy is (x^y)*ln(x) = (4.95^2.5)*ln(4.95) ~= 87.19.
  const double dzdy = std::pow(4.95, 2.5) * std::log(4.95);
  // By the chain rule, dz/dv is 27.53 * xgrad + 87.19 * ygrad
  EXPECT_DOUBLE_EQ(dzdx * 4.5 + dzdy * 0.0, z.derivatives()[0]);
  EXPECT_DOUBLE_EQ(dzdx * 1.1 + dzdy * 1.0, z.derivatives()[1]);
}

}  // namespace
}  // namespace math
}  // namespace drake
