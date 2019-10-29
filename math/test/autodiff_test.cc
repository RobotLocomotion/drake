#include "drake/math/autodiff.h"

#include <Eigen/Dense>
#include <gtest/gtest.h>

#include "drake/common/autodiff.h"
#include "drake/common/eigen_types.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_no_throw.h"
#include "drake/math/autodiff_gradient.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::AutoDiffScalar;

namespace drake {
namespace math {
namespace {

class AutodiffTest : public ::testing::Test {
 protected:
  typedef Eigen::AutoDiffScalar<VectorXd> Scalar;

  void SetUp() override {
    vec_.resize(2);

    // Set up to evaluate the derivatives at the values v0 and v1.
    vec_[0].value() = v0_;
    vec_[1].value() = v1_;

    // Provide enough room for differentiation with respect to both variables.
    vec_[0].derivatives().resize(2);
    vec_[1].derivatives().resize(2);

    // Herein, the shorthand notation is used: v0 = vec_[0] and v1 = vec_[1].
    // Set partial of v0 with respect to v0 (itself) to 1 (∂v0/∂v0 = 1).
    // Set partial of v0 with respect to v1 to 0 (∂v0/∂v1 = 0).
    vec_[0].derivatives()(0) = 1.0;
    vec_[0].derivatives()(1) = 0.0;

    // Set partial of v1 with respect to v0 to 0 (∂v1/∂v0 = 0).
    // Set partial of v1 with respect to v1 (itself) to 1 (∂v1/∂v1 = 1).
    vec_[1].derivatives()(0) = 0.0;
    vec_[1].derivatives()(1) = 1.0;

    // Do a calculation that is a function of variables v0 and v1.
    output_calculation_ = DoMath(vec_);
  }

  // Do a calculation involving real functions of two real variables. These
  // functions were chosen due to ease of differentiation.
  static VectorX<Scalar> DoMath(const VectorX<Scalar>& v) {
    VectorX<Scalar> output(3);
    // Shorthand notation: Denote v0 = v[0], v1 = v[1].
    // Function 0: y0 = cos(v0) + sin(v0) * cos(v0) / v1
    // Function 1: y1 = sin(v0) + v1.
    // Function 2: y2 = v0^2 + v1^3.
    output[0] = cos(v[0]) + sin(v[0]) * cos(v[0]) / v[1];
    output[1] = sin(v[0]) + v[1];
    output[2] = v[0] * v[0] + v[1] * v[1] * v[1];
    return output;
  }

  VectorX<Scalar> vec_;                 // Array of variables.
  VectorX<Scalar> output_calculation_;  // Functions that depend on variables.

  // Arbitrary values 7 and 9 will be used as test data.
  const double v0_ = 7.0;
  const double v1_ = 9.0;
};

// Tests that ToValueMatrix extracts the values from the autodiff.
TEST_F(AutodiffTest, ToValueMatrix) {
  const VectorXd values = autoDiffToValueMatrix(output_calculation_);
  VectorXd expected(3);
  expected[0] = cos(v0_) + sin(v0_) * cos(v0_) / v1_;
  expected[1] = sin(v0_) + v1_;
  expected[2] = v0_ * v0_ + v1_ * v1_ * v1_;
  EXPECT_TRUE(
      CompareMatrices(expected, values, 1e-10, MatrixCompareType::absolute))
      << values;
}

// Tests that ToGradientMatrix extracts the gradients from the autodiff.
TEST_F(AutodiffTest, ToGradientMatrix) {
  MatrixXd gradients = autoDiffToGradientMatrix(output_calculation_);

  MatrixXd expected(3, 2);
  // Shorthand notation: Denote v0 = vec_[0], v1 = vec_[1].
  // Function 0: y0 = cos(v0) + sin(v0) * cos(v0) / v1
  // Function 1: y1 = sin(v0) + v1.
  // Function 2: y2 = v0^2 + v1^3.
  // Calculate partial derivatives of y0, y1, y2 with respect to v0, v1.
  // ∂y0/∂v0 = -sin(v0) + (cos(v0)^2 - sin(v0)^2) / v1
  expected(0, 0) =
      -sin(v0_) + (cos(v0_) * cos(v0_) - sin(v0_) * sin(v0_)) / v1_;
  // ∂y0/∂v1 = -sin(v0) * cos(v0) / v1^2
  expected(0, 1) = -sin(v0_) * cos(v0_) / (v1_ * v1_);
  // ∂y1/∂v0 = cos(v0).
  expected(1, 0) = cos(v0_);
  // ∂y1/∂v1 = 1.
  expected(1, 1) = 1.0;
  // ∂y2/∂v0 = 2 * v0.
  expected(2, 0) = 2 * v0_;
  // ∂y2/∂v1 = 3 * v1^2.
  expected(2, 1) = 3 * v1_ * v1_;

  EXPECT_TRUE(
      CompareMatrices(expected, gradients, 1e-10, MatrixCompareType::absolute))
      << gradients;
}

GTEST_TEST(AdditionalAutodiffTest, DiscardGradient) {
  // Test the double case:
  Eigen::Matrix2d test = Eigen::Matrix2d::Identity();
  EXPECT_TRUE(CompareMatrices(DiscardGradient(test), test));

  Eigen::MatrixXd test2 = Eigen::Vector3d{1., 2., 3.};
  EXPECT_TRUE(CompareMatrices(DiscardGradient(test2), test2));

  // Test the AutoDiff case
  Vector3<AutoDiffXd> test3 = test2;
  // Note:  Neither of these would compile:
  //   Eigen::Vector3d test3out = test3;
  //   Eigen::Vector3d test3out = test3.cast<double>();
  // (so even compiling is a success).
  Eigen::Vector3d test3out = DiscardGradient(test3);
  EXPECT_TRUE(CompareMatrices(test3out, test2));

  Eigen::Isometry3d test5 = Eigen::Isometry3d::Identity();
  EXPECT_TRUE(CompareMatrices(DiscardGradient(test5).linear(), test5.linear()));
  EXPECT_TRUE(CompareMatrices(DiscardGradient(test5).translation(),
                              test5.translation()));

  Isometry3<AutoDiffXd> test6 = Isometry3<AutoDiffXd>::Identity();
  test6.translate(Vector3<AutoDiffXd>{3., 2., 1.});
  Eigen::Isometry3d test6b = DiscardGradient(test6);
  EXPECT_TRUE(CompareMatrices(test6b.linear(), Eigen::Matrix3d::Identity()));
  EXPECT_TRUE(
      CompareMatrices(test6b.translation(), Eigen::Vector3d{3., 2., 1.}));
}

GTEST_TEST(AdditionalAutodiffTest, DiscardZeroGradient) {
  // Test the double case:
  Eigen::Matrix2d test = Eigen::Matrix2d::Identity();
  DRAKE_EXPECT_NO_THROW(DiscardZeroGradient(test));
  EXPECT_TRUE(CompareMatrices(DiscardZeroGradient(test), test));

  Eigen::MatrixXd test2 = Eigen::Vector3d{1., 2., 3.};
  DRAKE_EXPECT_NO_THROW(DiscardZeroGradient(test2));
  EXPECT_TRUE(CompareMatrices(DiscardZeroGradient(test2), test2));
  // Check that the returned value is a reference to the original data.
  EXPECT_EQ(&DiscardZeroGradient(test2), &test2);

  // Test the AutoDiff case
  Eigen::Matrix<AutoDiffXd, 3, 1> test3 = test2;
  DRAKE_EXPECT_NO_THROW(DiscardZeroGradient(test3));
  // Note:  Neither of these would compile:
  //   Eigen::Vector3d test3out = test3;
  //   Eigen::Vector3d test3out = test3.cast<double>();
  // (so even compiling is a success).
  Eigen::Vector3d test3out = DiscardZeroGradient(test3);
  EXPECT_TRUE(CompareMatrices(test3out, test2));
  test3 =
      initializeAutoDiffGivenGradientMatrix(test2, Eigen::MatrixXd::Zero(3, 2));
  EXPECT_TRUE(CompareMatrices(DiscardZeroGradient(test3), test2));
  test3 =
      initializeAutoDiffGivenGradientMatrix(test2, Eigen::MatrixXd::Ones(3, 2));
  EXPECT_THROW(DiscardZeroGradient(test3), std::runtime_error);
  DRAKE_EXPECT_NO_THROW(DiscardZeroGradient(test3, 2.));

  Eigen::Isometry3d test5 = Eigen::Isometry3d::Identity();
  DRAKE_EXPECT_NO_THROW(DiscardZeroGradient(test5));
  EXPECT_TRUE(
      CompareMatrices(DiscardZeroGradient(test5).linear(), test5.linear()));
  EXPECT_TRUE(CompareMatrices(DiscardZeroGradient(test5).translation(),
                              test5.translation()));
  // Check that the returned value is a reference to the original data.
  EXPECT_EQ(&DiscardZeroGradient(test5), &test5);

  Isometry3<AutoDiffXd> test6 = Isometry3<AutoDiffXd>::Identity();
  test6.translate(Vector3<AutoDiffXd>{3., 2., 1.});
  DRAKE_EXPECT_NO_THROW(DiscardZeroGradient(test5));
  Eigen::Isometry3d test6b = DiscardZeroGradient(test6);
  EXPECT_TRUE(CompareMatrices(test6b.linear(), Eigen::Matrix3d::Identity()));
  EXPECT_TRUE(
      CompareMatrices(test6b.translation(), Eigen::Vector3d{3., 2., 1.}));
  test6.linear()(0, 0).derivatives() = Vector3d{1., 2., 3.};

  EXPECT_THROW(DiscardZeroGradient(test6), std::runtime_error);
}

// Make sure that casting to autodiff always results in zero gradients.
GTEST_TEST(AdditionalAutodiffTest, CastToAutoDiff) {
  Vector2<AutoDiffXd> dynamic = Vector2d::Ones().cast<AutoDiffXd>();
  const auto dynamic_gradients = autoDiffToGradientMatrix(dynamic);
  EXPECT_EQ(dynamic_gradients.rows(), 2);
  EXPECT_EQ(dynamic_gradients.cols(), 0);

  using VectorUpTo16d = Eigen::Matrix<double, Eigen::Dynamic, 1, 0, 16, 1>;
  using AutoDiffUpTo16d = Eigen::AutoDiffScalar<VectorUpTo16d>;
  Vector2<AutoDiffUpTo16d> dynamic_max =
      Vector2d::Ones().cast<AutoDiffUpTo16d>();
  const auto dynamic_max_gradients = autoDiffToGradientMatrix(dynamic_max);
  EXPECT_EQ(dynamic_max_gradients.rows(), 2);
  EXPECT_EQ(dynamic_max_gradients.cols(), 0);

  Vector2<AutoDiffScalar<Vector3d>> fixed =
      Vector2d::Ones().cast<AutoDiffScalar<Vector3d>>();
  const auto fixed_gradients = autoDiffToGradientMatrix(fixed);
  EXPECT_EQ(fixed_gradients.rows(), 2);
  EXPECT_EQ(fixed_gradients.cols(), 3);
  EXPECT_TRUE(fixed_gradients.isZero(0.));
}

}  // namespace
}  // namespace math
}  // namespace drake
