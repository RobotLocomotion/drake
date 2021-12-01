#include "drake/math/autodiff.h"

#include <Eigen/Dense>
#include <gtest/gtest.h>

#include "drake/common/autodiff.h"
#include "drake/common/eigen_types.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_no_throw.h"
#include "drake/common/test_utilities/expect_throws_message.h"
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

TEST_F(AutodiffTest, ExtractValue) {
  const VectorXd values = ExtractValue(output_calculation_);
  VectorXd expected(3);
  expected[0] = cos(v0_) + sin(v0_) * cos(v0_) / v1_;
  expected[1] = sin(v0_) + v1_;
  expected[2] = v0_ * v0_ + v1_ * v1_ * v1_;
  EXPECT_TRUE(
      CompareMatrices(expected, values, 1e-10, MatrixCompareType::absolute))
      << values;

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
  EXPECT_TRUE(CompareMatrices(autoDiffToValueMatrix(output_calculation_),
      values));
#pragma GCC diagnostic pop
}

TEST_F(AutodiffTest, ExtractGradient) {
  MatrixXd gradients = ExtractGradient(output_calculation_);

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

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
  EXPECT_TRUE(CompareMatrices(autoDiffToGradientMatrix(output_calculation_),
      gradients));
#pragma GCC diagnostic pop

  // Given an AutoDiff matrix with no derivatives, ExtractGradient() should
  // return a matrix with zero-length rows, or return with specified-length
  // rows of all zeroes.
  Vector3<AutoDiffXd> no_derivatives;
  const auto gradient_with_no_derivs = ExtractGradient(no_derivatives);
  EXPECT_EQ(gradient_with_no_derivs.rows(), 3);
  EXPECT_EQ(gradient_with_no_derivs.cols(), 0);

  const auto gradient_with_three_derivs = ExtractGradient(no_derivatives, 3);
  EXPECT_TRUE(
      CompareMatrices(gradient_with_three_derivs, Eigen::Matrix3d::Zero()));

  // Create an AutoDiff matrix with mixed zero-length and consistent, non-zero
  // length derivatives. ExtractGradient() should be happy with this.
  Vector3<AutoDiffXd> mixed_good;
  mixed_good(0) = AutoDiffXd(1.0, VectorXd::Zero(3));
  mixed_good(1) = AutoDiffXd(2.0, VectorXd());
  mixed_good(2) = AutoDiffXd(3.0, VectorXd::Zero(3));
  const auto mixed_good_gradient = ExtractGradient(mixed_good);
  EXPECT_TRUE(CompareMatrices(mixed_good_gradient, Eigen::Matrix3d::Zero()));

  // Now specify a number of derivatives that matches the actual number.
  EXPECT_NO_THROW(ExtractGradient(mixed_good, 3));

  // But we should fail if the specified size doesn't match actual.
  DRAKE_EXPECT_THROWS_MESSAGE(ExtractGradient(mixed_good, 19),
      "ExtractGradient..: Input matrix has 3.*but.*specified.*19.*"
      "should have zero.*or.*should match.*");

  // Reject an AutoDiff matrix with inconsistent non-zero length derivatives.
  Vector3<AutoDiffXd> mixed_bad;
  mixed_bad(0) = AutoDiffXd(1.0, VectorXd::Zero(3));
  mixed_bad(1) = AutoDiffXd(2.0, VectorXd());
  mixed_bad(2) = AutoDiffXd(3.0, VectorXd::Zero(2));
  DRAKE_EXPECT_THROWS_MESSAGE(ExtractGradient(mixed_bad),
      "ExtractGradient..: Input matrix.*inconsistent.*3 and 2.*");
}

// Check the various overloads of InitializeAutoDiff(value). There are a
// variety of template parameters and various behaviors depending on whether
// the underlying Eigen matrices are fixed- or dynamic-sized. Sugar functions
// are available that take common defaults and give results in the function
// return rather than into a pre-allocated output argument.
//
// In some cases we also need to verify that the template meta programming
// yields the expected types, since that is part of the function documentation.
GTEST_TEST(AdditionalAutodiffTest, InitializeNoGradientMatrix) {
  const auto value = (Eigen::Matrix2d() <<  1.0, 2.0,
                                            3.0, 4.0).finished();

  // Fixed-size value, fixed-size gradient.
  Eigen::Matrix<Eigen::AutoDiffScalar<Eigen::Vector4d>, 2, 2> autodiff2;
  // This is the general method. All the other no-gradient-methods call it.
  InitializeAutoDiff(value, {}, {}, &autodiff2);
  EXPECT_TRUE(CompareMatrices(ExtractValue(autodiff2), value));
  EXPECT_TRUE(CompareMatrices(ExtractGradient(autodiff2),
      Eigen::Matrix4d::Identity()));

  // Cursory check of overload that defaults the middle two parameters, exactly
  // equivalent to the more-general signature as invoked above.
  Eigen::Matrix<Eigen::AutoDiffScalar<Eigen::Vector4d>, 2, 2> autodiff22;
  InitializeAutoDiff(2 * value, &autodiff22);
  EXPECT_TRUE(CompareMatrices(ExtractValue(autodiff22), 2 * value));
  EXPECT_TRUE(CompareMatrices(ExtractGradient(autodiff22),
      Eigen::Matrix4d::Identity()));

  // Even simpler overload that defaults the middle parameters and puts the
  // result in the return value. Derivatives are fixed size if a size is
  // specified as a template argument, dynamic otherwise.

  const auto ad4_return = InitializeAutoDiff<4>(value);  // Fixed derivatives.
  // Since value was fixed size, ad_return should be also.
  EXPECT_EQ(decltype(ad4_return)::ColsAtCompileTime, 2);
  EXPECT_EQ(decltype(ad4_return)::RowsAtCompileTime, 2);
  EXPECT_EQ(decltype(ad4_return)::Scalar::DerType::RowsAtCompileTime, 4);
  EXPECT_TRUE(CompareMatrices(ExtractValue(ad4_return), value));
  EXPECT_TRUE(CompareMatrices(ExtractGradient(ad4_return),
                              Eigen::Matrix4d::Identity()));

  const auto adX_return = InitializeAutoDiff(value);  // Dynamic derivatives.
  // Since value was fixed size, adX_return should be also.
  EXPECT_EQ(decltype(adX_return)::ColsAtCompileTime, 2);
  EXPECT_EQ(decltype(adX_return)::RowsAtCompileTime, 2);
  EXPECT_EQ(decltype(adX_return)::Scalar::DerType::RowsAtCompileTime,
            Eigen::Dynamic);  // But derivatives are dynamically sized.
  EXPECT_TRUE(CompareMatrices(ExtractValue(adX_return), value));
  EXPECT_TRUE(CompareMatrices(ExtractGradient(adX_return),
                              Eigen::Matrix4d::Identity()));

  // Fixed-size value, variable-size gradient.
  Eigen::Matrix<AutoDiffXd, 2, 2> autodiffX;
  InitializeAutoDiff(value, {}, {}, &autodiffX);
  EXPECT_TRUE(CompareMatrices(ExtractValue(autodiffX), value));
  EXPECT_TRUE(CompareMatrices(ExtractGradient(autodiffX),
                              Eigen::MatrixXd::Identity(4, 4)));

  // Check that the general method obeys explicit size & start arguments
  // for the derivatives.
  InitializeAutoDiff(value, 7, 3, &autodiffX);
  EXPECT_TRUE(CompareMatrices(ExtractGradient(autodiffX).block<4, 3>(0, 0),
                              Eigen::Matrix<double, 4, 3>::Zero()));
  EXPECT_TRUE(CompareMatrices(ExtractGradient(autodiffX).block<4, 4>(0, 3),
                              Eigen::Matrix4d::Identity()));

  // The function return variant can also take specified values for size and
  // start position within the gradient. We trust it just passes these through
  // to the general method, but verify anyway.
  const auto ad_size = InitializeAutoDiff(value, 6, 2);
  EXPECT_TRUE(CompareMatrices(ExtractValue(ad_size), value));
  EXPECT_TRUE(CompareMatrices(ExtractGradient(ad_size).block<4, 2>(0, 0),
                              Eigen::Matrix<double, 4, 2>::Zero()));
  EXPECT_TRUE(CompareMatrices(ExtractGradient(ad_size).block<4, 4>(0, 2),
                              Eigen::Matrix4d::Identity()));

  // Check that a dynamic-size value yields a dynamic-size result.
  const MatrixXd dynamic_value(value);
  const auto dynamic_result = InitializeAutoDiff(dynamic_value);
  EXPECT_EQ(decltype(dynamic_result)::ColsAtCompileTime, Eigen::Dynamic);
  EXPECT_EQ(decltype(dynamic_result)::RowsAtCompileTime, Eigen::Dynamic);
  EXPECT_EQ(dynamic_result.cols(), 2);
  EXPECT_EQ(dynamic_result.rows(), 2);
}

// Check the two overloads of InitializeAutoDiff(value, gradient). The
// most-general method takes an output argument for the result; a sugar method
// uses that to give the result as the function return.
GTEST_TEST(AdditionalAutodiffTest, InitializeWithGradientMatrix) {
  const Eigen::Matrix2d value = (Eigen::Matrix2d() <<  1.0, 2.0,
                                                       3.0, 4.0).finished();
  const Eigen::Matrix4d gradient = 2 * Eigen::Matrix4d::Identity();

  // Fixed-size value, fixed-size gradient.
  Eigen::Matrix<Eigen::AutoDiffScalar<Eigen::Vector4d>, 2, 2> autodiff2;
  // This is the general method. The other (value, gradient) signature just
  // calls it.
  InitializeAutoDiff(value, gradient, &autodiff2);
  EXPECT_TRUE(CompareMatrices(ExtractValue(autodiff2), value));
  EXPECT_TRUE(CompareMatrices(ExtractGradient(autodiff2), gradient));

  // The other signature should behave identically, including meta-computing a
  // fixed-size return type.
  const auto ad2_return = InitializeAutoDiff(value, gradient);
  EXPECT_EQ(decltype(ad2_return)::ColsAtCompileTime, 2);
  EXPECT_EQ(decltype(ad2_return)::RowsAtCompileTime, 2);
  EXPECT_EQ(decltype(ad2_return)::Scalar::DerType::RowsAtCompileTime, 4);
  EXPECT_TRUE(CompareMatrices(ExtractValue(ad2_return), value));
  EXPECT_TRUE(CompareMatrices(ExtractGradient(ad2_return), gradient));

  // Repeat the last two tests using dynamically-sized matrices. This should
  // produce dynamically-sized results.
  const Eigen::MatrixXd dynamic_value(value);
  const Eigen::MatrixXd dynamic_gradient(gradient);

  MatrixX<AutoDiffXd> autodiff;
  InitializeAutoDiff(dynamic_value, dynamic_gradient, &autodiff);
  EXPECT_EQ(autodiff.cols(), 2);
  EXPECT_EQ(autodiff.rows(), 2);
  EXPECT_EQ(autodiff(0, 0).derivatives().size(), 4);
  EXPECT_TRUE(CompareMatrices(ExtractValue(autodiff), value));
  EXPECT_TRUE(CompareMatrices(ExtractGradient(autodiff), gradient));

  const auto autodiff_return =
      InitializeAutoDiff(dynamic_value, dynamic_gradient);
  EXPECT_EQ(decltype(autodiff_return)::ColsAtCompileTime, Eigen::Dynamic);
  EXPECT_EQ(decltype(autodiff_return)::RowsAtCompileTime, Eigen::Dynamic);
  EXPECT_EQ(decltype(autodiff_return)::Scalar::DerType::RowsAtCompileTime,
            Eigen::Dynamic);
  EXPECT_EQ(autodiff_return.cols(), 2);
  EXPECT_EQ(autodiff_return.rows(), 2);
  EXPECT_EQ(autodiff_return(0, 0).derivatives().size(), 4);
  EXPECT_TRUE(CompareMatrices(ExtractValue(autodiff_return), value));
  EXPECT_TRUE(CompareMatrices(ExtractGradient(autodiff_return), gradient));
}

GTEST_TEST(AdditionalAutodiffTest, InitializeAutoDiffTuple) {
  // When all sizes are known at compile time, the resulting
  // AutoDiffScalars should have a compile time number of
  // derivatives.
  const Eigen::Matrix2d matrix2 = Eigen::Matrix2d::Identity();
  const Eigen::Vector3d vec3{1., 2., 3.};
  const Eigen::Vector4d vec4{5., 6., 7., 8.};

  const auto tuple = math::InitializeAutoDiffTuple(matrix2, vec3, vec4);
  EXPECT_EQ(std::tuple_size_v<decltype(tuple)>, 3);

  EXPECT_EQ(std::get<0>(tuple).rows(), 2);
  EXPECT_EQ(std::get<0>(tuple).cols(), 2);
  EXPECT_EQ(std::get<1>(tuple).size(), 3);
  EXPECT_EQ(std::get<2>(tuple).size(), 4);

  // This is the expected type of the derivatives vector (in every element).
  const Eigen::Matrix<double, 11, 1>& deriv_12 =
      std::get<1>(tuple).coeffRef(2).derivatives();
  // Check that we didn't create a new copy (i.e. we got the right type).
  EXPECT_EQ(&deriv_12, &std::get<1>(tuple).coeffRef(2).derivatives());

  // Since vec3[2] is the 7th variable, we expect only element 7 of its
  // derivatives vector to be 1.
  VectorXd expected(11);
  expected << 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0;
  EXPECT_EQ(deriv_12, expected);

  // Now let's replace vec3 with a dynamic version of the same size.
  Eigen::VectorXd vec3d(3);
  vec3d << 10., 11., 12.;

  const auto tupled = math::InitializeAutoDiffTuple(matrix2, vec3d, vec4);
  EXPECT_EQ(std::tuple_size_v<decltype(tupled)>, 3);

  EXPECT_EQ(std::get<0>(tupled).rows(), 2);
  EXPECT_EQ(std::get<0>(tupled).cols(), 2);
  EXPECT_EQ(std::get<1>(tupled).size(), 3);
  EXPECT_EQ(std::get<2>(tupled).size(), 4);

  // This is the expected type of the derivatives vector (in every element).
  const Eigen::Matrix<double, Eigen::Dynamic, 1>& deriv_12d =
      std::get<1>(tupled).coeffRef(2).derivatives();
  // Check that we didn't create a new copy (i.e. we got the right type).
  EXPECT_EQ(&deriv_12d, &std::get<1>(tupled).coeffRef(2).derivatives());

  // We should still get the same value at run time.
  expected << 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0;
  EXPECT_EQ(deriv_12d, expected);
}

// See note in class documentation for our AutoDiffXd specialization in
// common/autodiffxd.h for why we initialize the value field even though
// that is not part of the Eigen::AutoDiffScalar contract.
GTEST_TEST(AdditionalAutodiffTest, ValueIsInitializedToNaN) {
  AutoDiffXd autodiff;
  EXPECT_TRUE(std::isnan(autodiff.value()));
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
  test3 = InitializeAutoDiff(test2, Eigen::MatrixXd::Zero(3, 2));
  EXPECT_TRUE(CompareMatrices(DiscardZeroGradient(test3), test2));
  test3 = InitializeAutoDiff(test2, Eigen::MatrixXd::Ones(3, 2));

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
  EXPECT_TRUE(CompareMatrices(
      initializeAutoDiffGivenGradientMatrix(test2, Eigen::MatrixXd::Ones(3, 2)),
      test3));
#pragma GCC diagnostic pop

  EXPECT_THROW(DiscardZeroGradient(test3), std::runtime_error);
  DRAKE_EXPECT_NO_THROW(DiscardZeroGradient(test3, 2.));
}

// Make sure that casting to autodiff always results in zero gradients.
GTEST_TEST(AdditionalAutodiffTest, CastToAutoDiff) {
  Vector2<AutoDiffXd> dynamic = Vector2d::Ones().cast<AutoDiffXd>();
  const auto dynamic_gradients = ExtractGradient(dynamic);
  EXPECT_EQ(dynamic_gradients.rows(), 2);
  EXPECT_EQ(dynamic_gradients.cols(), 0);

  using VectorUpTo16d = Eigen::Matrix<double, Eigen::Dynamic, 1, 0, 16, 1>;
  using AutoDiffUpTo16d = Eigen::AutoDiffScalar<VectorUpTo16d>;
  Vector2<AutoDiffUpTo16d> dynamic_max =
      Vector2d::Ones().cast<AutoDiffUpTo16d>();
  const auto dynamic_max_gradients = ExtractGradient(dynamic_max);
  EXPECT_EQ(dynamic_max_gradients.rows(), 2);
  EXPECT_EQ(dynamic_max_gradients.cols(), 0);

  Vector2<AutoDiffScalar<Vector3d>> fixed =
      Vector2d::Ones().cast<AutoDiffScalar<Vector3d>>();
  const auto fixed_gradients = ExtractGradient(fixed);
  EXPECT_EQ(fixed_gradients.rows(), 2);
  EXPECT_EQ(fixed_gradients.cols(), 3);
  EXPECT_TRUE(fixed_gradients.isZero(0.));
}

GTEST_TEST(GetDerivativeSize, Test) {
  // Empty gradient.
  EXPECT_EQ(GetDerivativeSize(Eigen::Vector2d(1, 2).cast<AutoDiffXd>()), 0);
  // Non-empty gradient.
  EXPECT_EQ(GetDerivativeSize(InitializeAutoDiff(
                Eigen::Vector2d(1, 2), Eigen::Matrix<double, 2, 3>::Ones())),
            3);
  // Some derivatives have empty size.
  Eigen::Matrix<AutoDiffXd, 3, 1> x;
  x(0).value() = 0;
  x(0).derivatives() = Eigen::VectorXd(0);
  x(1).value() = 1;
  x(1).derivatives() = Eigen::Vector4d::Ones();
  x(2).value() = 2;
  x(2).derivatives() = Eigen::VectorXd::Ones(4);
  EXPECT_EQ(GetDerivativeSize(x), 4);

  // Inconsistent derivative size.
  x(2).derivatives() = Eigen::VectorXd::Ones(3);
  DRAKE_EXPECT_THROWS_MESSAGE(GetDerivativeSize(x),
                              ".* has size 3, while another entry has size 4");
}
}  // namespace
}  // namespace math
}  // namespace drake

