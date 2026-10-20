#include "drake/math/autodiff.h"

#include <Eigen/Dense>
#include <gtest/gtest.h>
#include <unsupported/Eigen/AutoDiff>

#include "drake/common/autodiff.h"
#include "drake/common/eigen_types.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_no_throw.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/common/test_utilities/limit_malloc.h"
#include "drake/math/autodiff_gradient.h"

using Eigen::AutoDiffScalar;
using Eigen::Matrix2d;
using Eigen::MatrixXd;
using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::VectorXd;
using Vector5d = Eigen::Matrix<double, 5, 1>;

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
  VectorXd expected(3);
  expected[0] = cos(v0_) + sin(v0_) * cos(v0_) / v1_;
  expected[1] = sin(v0_) + v1_;
  expected[2] = v0_ * v0_ + v1_ * v1_ * v1_;

  // Overload that returns the result.
  const VectorXd values_return = ExtractValue(output_calculation_);
  const double kTol = 1e-10;
  EXPECT_TRUE(CompareMatrices(expected, values_return, kTol,
                              MatrixCompareType::absolute));

  // Overload that writes the result to a provided output vector.
  VectorXd values_out;
  ExtractValue(output_calculation_, &values_out);
  EXPECT_TRUE(
      CompareMatrices(expected, values_out, kTol, MatrixCompareType::absolute));

  // Test writing result to a preallocated output vector.
  VectorXd values_prealloc(3);
  {
    test::LimitMalloc guard;
    ExtractValue(output_calculation_, &values_prealloc);
  }
  EXPECT_TRUE(CompareMatrices(expected, values_prealloc, kTol,
                              MatrixCompareType::absolute));
}

TEST_F(AutodiffTest, ExtractGradient) {
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

  // Overload that returns the result.
  const MatrixXd gradients_return = ExtractGradient(output_calculation_);
  const double kTol = 1e-10;
  EXPECT_TRUE(CompareMatrices(expected, gradients_return, kTol,
                              MatrixCompareType::absolute));

  // Overload that writes the result to a provided output matrix.
  MatrixXd gradients_out;
  ExtractGradient(output_calculation_, {}, &gradients_out);
  EXPECT_TRUE(CompareMatrices(expected, gradients_out, kTol,
                              MatrixCompareType::absolute));

  // Test writing result to a preallocated output matrix.
  MatrixXd gradients_prealloc(output_calculation_.size(), 2);
  {
    test::LimitMalloc guard;
    ExtractGradient(output_calculation_, {}, &gradients_prealloc);
  }
  EXPECT_TRUE(CompareMatrices(expected, gradients_prealloc, kTol,
                              MatrixCompareType::absolute));

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

  // Entries with no derivatives are written as zeros, both before and after the
  // first entry with derivatives. The output starts with the wrong shape but
  // the same size, so resizing it keeps the stale values.
  VectorX<AutoDiffXd> mixed_sparse(4);
  mixed_sparse(0) = AutoDiffXd(1.0);
  mixed_sparse(1) = AutoDiffXd(2.0, Vector2d(1.0, 2.0));
  mixed_sparse(2) = AutoDiffXd(3.0);
  mixed_sparse(3) = AutoDiffXd(4.0, Vector2d(3.0, 4.0));
  MatrixXd mixed_sparse_expected(4, 2);
  mixed_sparse_expected << 0, 0, 1, 2, 0, 0, 3, 4;
  for (const std::optional<int> num_derivatives :
       {std::optional<int>(), std::optional<int>(2)}) {
    MatrixXd mixed_sparse_gradient = MatrixXd::Constant(2, 4, 99.0);
    ExtractGradient(mixed_sparse, num_derivatives, &mixed_sparse_gradient);
    EXPECT_TRUE(CompareMatrices(mixed_sparse_gradient, mixed_sparse_expected));
  }

  // But we should fail if the specified size doesn't match actual.
  DRAKE_EXPECT_THROWS_MESSAGE(
      ExtractGradient(mixed_good, 19),
      "ExtractGradient..: Input matrix has 3.*but.*specified.*19.*"
      "should have zero.*or.*should match.*");
  // Such a mismatch is detected before the output is modified.
  Eigen::Matrix3Xd mixed_good_gradient_out =
      Eigen::Matrix3Xd::Constant(3, 1, 99);
  EXPECT_THROW(ExtractGradient(mixed_good, 19, &mixed_good_gradient_out),
               std::exception);
  EXPECT_TRUE(CompareMatrices(mixed_good_gradient_out,
                              Eigen::Matrix3Xd::Constant(3, 1, 99)));

  // Reject an AutoDiff matrix with inconsistent non-zero length derivatives.
  Vector3<AutoDiffXd> mixed_bad;
  mixed_bad(0) = AutoDiffXd(1.0, VectorXd::Zero(3));
  mixed_bad(1) = AutoDiffXd(2.0, VectorXd());
  mixed_bad(2) = AutoDiffXd(3.0, VectorXd::Zero(2));
  DRAKE_EXPECT_THROWS_MESSAGE(
      ExtractGradient(mixed_bad),
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
  const auto value = (Eigen::Matrix2d() << 1.0, 2.0, 3.0, 4.0).finished();

  // Fixed-size value, fixed-size gradient.
  Eigen::Matrix<Eigen::AutoDiffScalar<Eigen::Vector4d>, 2, 2> autodiff2;
  // This is the general method. All the other no-gradient-methods call it.
  InitializeAutoDiff(value, {}, {}, &autodiff2);
  EXPECT_TRUE(CompareMatrices(ExtractValue(autodiff2), value));
  EXPECT_TRUE(
      CompareMatrices(ExtractGradient(autodiff2), Eigen::Matrix4d::Identity()));

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

  // Row-major order is preserved through InitializeAutoDiff and
  // ExtractGradient.
  Eigen::Matrix<double, 2, 3, Eigen::RowMajor> value_rowmajor;
  // clang-format off
  value_rowmajor <<
      1.0, 2.0, 3.0,
      4.0, 5.0, 6.0;
  // clang-format on
  const auto adXT_return = InitializeAutoDiff(value_rowmajor);
  EXPECT_TRUE(decltype(adXT_return)::IsRowMajor);
  EXPECT_EQ(decltype(adXT_return)::RowsAtCompileTime, 2);
  EXPECT_EQ(decltype(adXT_return)::ColsAtCompileTime, 3);
  EXPECT_TRUE(CompareMatrices(ExtractValue(adXT_return), value_rowmajor));
  EXPECT_TRUE(
      CompareMatrices(ExtractGradient(adXT_return), MatrixXd::Identity(6, 6)));

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
  const Eigen::Matrix2d value =
      (Eigen::Matrix2d() << 1.0, 2.0, 3.0, 4.0).finished();
  const Eigen::Matrix4d gradient = 2 * Eigen::Matrix4d::Identity();

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

// Returns the number of heap allocations made while calling `func`.
template <typename Func>
int CountAllocations(const Func& func) {
  test::LimitMalloc guard({.max_num_allocations = -1});
  func();
  return guard.num_allocations();
}

// Reading a coefficient of an Eigen product expression evaluates the entire
// product. These tests pass product expressions to the functions under test
// and check that each product is evaluated only once: the call must make
// exactly as many heap allocations as evaluating the products by hand and then
// calling the function on the results. Evaluating once per row would instead
// add (at least) one allocation per row.
GTEST_TEST(AdditionalAutodiffTest, InitializeNoGradientMatrixFromProduct) {
  // Dynamic size.
  const int num_rows = 10;
  const MatrixXd A = MatrixXd::Random(num_rows, 4);
  const VectorXd x = VectorXd::Random(4);
  const VectorXd value = A * x;
  VectorX<AutoDiffXd> autodiff(num_rows);
  const int num_allocations_evaluated = CountAllocations([&]() {
    const VectorXd value_evaluated = A * x;
    InitializeAutoDiff(value_evaluated, &autodiff);
  });
  const int num_allocations_product = CountAllocations([&]() {
    InitializeAutoDiff(A * x, &autodiff);
  });
  EXPECT_EQ(num_allocations_product, num_allocations_evaluated);
  EXPECT_TRUE(CompareMatrices(ExtractValue(autodiff), value));
  EXPECT_TRUE(CompareMatrices(ExtractGradient(autodiff),
                              MatrixXd::Identity(num_rows, num_rows)));

  // Static size. The product is evaluated on the stack, so with fixed-size
  // derivatives nothing is allocated at all.
  const Eigen::Matrix3d R = Eigen::Matrix3d::Random();
  const Vector3d p = Vector3d::Random();
  Eigen::Matrix<Eigen::AutoDiffScalar<Vector3d>, 3, 1> autodiff_static;
  {
    test::LimitMalloc guard;
    InitializeAutoDiff(R * p, &autodiff_static);
  }
  EXPECT_TRUE(CompareMatrices(ExtractValue(autodiff_static), R * p));
  EXPECT_TRUE(CompareMatrices(ExtractGradient(autodiff_static),
                              Eigen::Matrix3d::Identity()));
}

GTEST_TEST(AdditionalAutodiffTest, InitializeWithGradientMatrixFromProduct) {
  // Dynamic size, e.g., a constraint's value A * x and its gradient
  // A * J * dx, chained through the gradient dx of the decision variables.
  const int num_rows = 10;
  const MatrixXd A = MatrixXd::Random(num_rows, 4);
  const VectorXd x = VectorXd::Random(4);
  const MatrixXd J = MatrixXd::Random(4, 5);
  const MatrixXd dx = MatrixXd::Random(5, 6);
  const VectorXd value = A * x;
  const MatrixXd gradient = A * J * dx;
  VectorX<AutoDiffXd> autodiff(num_rows);
  const int num_allocations_evaluated = CountAllocations([&]() {
    const VectorXd value_evaluated = A * x;
    const MatrixXd gradient_evaluated = A * J * dx;
    InitializeAutoDiff(value_evaluated, gradient_evaluated, &autodiff);
  });
  const int num_allocations_product = CountAllocations([&]() {
    InitializeAutoDiff(A * x, A * J * dx, &autodiff);
  });
  EXPECT_EQ(num_allocations_product, num_allocations_evaluated);
  EXPECT_TRUE(CompareMatrices(ExtractValue(autodiff), value));
  EXPECT_TRUE(CompareMatrices(ExtractGradient(autodiff), gradient, 1e-14));

  // Static size. The products are evaluated on the stack, so they allocate
  // nothing beyond what plain fixed-size inputs do.
  const Eigen::Matrix3d R = Eigen::Matrix3d::Random();
  const Vector3d p = Vector3d::Random();
  const Eigen::Matrix3d Jp = Eigen::Matrix3d::Random();
  const Vector3d value_static = R * p;
  const Eigen::Matrix3d gradient_static = R * Jp;
  Vector3<AutoDiffXd> autodiff_static;
  InitializeAutoDiff(value_static, gradient_static, &autodiff_static);
  const int num_allocations_plain = CountAllocations([&]() {
    InitializeAutoDiff(value_static, gradient_static, &autodiff_static);
  });
  const int num_allocations_static_product = CountAllocations([&]() {
    InitializeAutoDiff(R * p, R * Jp, &autodiff_static);
  });
  EXPECT_EQ(num_allocations_static_product, num_allocations_plain);
  EXPECT_TRUE(CompareMatrices(ExtractValue(autodiff_static), value_static));
  EXPECT_TRUE(
      CompareMatrices(ExtractGradient(autodiff_static), gradient_static));
}

// Products are not the only such expressions; reading a coefficient of a
// Solve expression likewise performs the entire solve.
GTEST_TEST(AdditionalAutodiffTest, InitializeWithGradientMatrixFromSolve) {
  const int num_rows = 10;
  const MatrixXd M = MatrixXd::Random(num_rows, num_rows);
  const MatrixXd S =
      M * M.transpose() + num_rows * MatrixXd::Identity(num_rows, num_rows);
  const VectorXd b = VectorXd::Random(num_rows);
  const MatrixXd B = MatrixXd::Random(num_rows, 6);
  // Factor outside of the counted calls, so they count only the solves.
  const Eigen::LLT<MatrixXd> llt(S);
  const VectorXd value = llt.solve(b);
  const MatrixXd gradient = llt.solve(B);
  VectorX<AutoDiffXd> autodiff(num_rows);
  const int num_allocations_evaluated = CountAllocations([&]() {
    const VectorXd value_evaluated = llt.solve(b);
    const MatrixXd gradient_evaluated = llt.solve(B);
    InitializeAutoDiff(value_evaluated, gradient_evaluated, &autodiff);
  });
  const int num_allocations_solve = CountAllocations([&]() {
    InitializeAutoDiff(llt.solve(b), llt.solve(B), &autodiff);
  });
  EXPECT_EQ(num_allocations_solve, num_allocations_evaluated);
  EXPECT_TRUE(CompareMatrices(ExtractValue(autodiff), value));
  EXPECT_TRUE(CompareMatrices(ExtractGradient(autodiff), gradient));
}

// A gradient with zero columns means "no derivatives". This happens in
// practice when a constraint's decision variables carry no derivatives, so
// that the chained gradient dxdot_dx * dx_dvars has no columns.
GTEST_TEST(AdditionalAutodiffTest, InitializeWithEmptyGradientMatrix) {
  const int num_rows = 3;
  const MatrixXd A = MatrixXd::Random(num_rows, 4);
  const VectorXd x = VectorXd::Random(4);
  const MatrixXd J = MatrixXd::Random(4, 0);
  const VectorXd value = A * x;

  // Both as a plain matrix and as an unevaluated product, which the helper
  // evaluates into an empty temporary whose data pointer is null.
  const MatrixXd gradient_plain = A * J;
  VectorX<AutoDiffXd> autodiff(num_rows);
  InitializeAutoDiff(value, gradient_plain, &autodiff);
  for (int i = 0; i < num_rows; ++i) {
    EXPECT_EQ(autodiff(i).value(), value(i));
    EXPECT_EQ(autodiff(i).derivatives().size(), 0);
  }
  InitializeAutoDiff(A * x, A * J, &autodiff);
  for (int i = 0; i < num_rows; ++i) {
    EXPECT_EQ(autodiff(i).value(), value(i));
    EXPECT_EQ(autodiff(i).derivatives().size(), 0);
  }
}

// A product of a double matrix and an AutoDiff vector is an unevaluated Eigen
// expression. (A product of two AutoDiff matrices is evaluated eagerly by
// Drake's AutoDiff operator*, so it is not interesting here.)
GTEST_TEST(AdditionalAutodiffTest, ExtractFromProduct) {
  // Dynamic size.
  const int num_rows = 10;
  const MatrixXd A = MatrixXd::Random(num_rows, 4);
  const VectorXd x_value = VectorXd::Random(4);
  const VectorX<AutoDiffXd> x = InitializeAutoDiff(x_value);
  VectorXd value(num_rows);
  MatrixXd gradient(num_rows, 4);
  const int num_value_allocations_evaluated = CountAllocations([&]() {
    const VectorX<AutoDiffXd> y = A * x;
    ExtractValue(y, &value);
  });
  const int num_value_allocations_product = CountAllocations([&]() {
    ExtractValue(A * x, &value);
  });
  EXPECT_EQ(num_value_allocations_product, num_value_allocations_evaluated);
  EXPECT_TRUE(CompareMatrices(value, A * x_value, 1e-14));

  const int num_gradient_allocations_evaluated = CountAllocations([&]() {
    const VectorX<AutoDiffXd> y = A * x;
    ExtractGradient(y, {}, &gradient);
  });
  const int num_gradient_allocations_product = CountAllocations([&]() {
    ExtractGradient(A * x, {}, &gradient);
  });
  EXPECT_EQ(num_gradient_allocations_product,
            num_gradient_allocations_evaluated);
  EXPECT_TRUE(CompareMatrices(gradient, A, 1e-14));

  // Static size.
  const Eigen::Matrix3d R = Eigen::Matrix3d::Random();
  const Vector3d p_value = Vector3d::Random();
  const Vector3<AutoDiffXd> p = InitializeAutoDiff(p_value);
  Vector3d value_static;
  Eigen::Matrix3Xd gradient_static(3, 3);
  const int num_static_allocations_evaluated = CountAllocations([&]() {
    const Vector3<AutoDiffXd> y_for_value = R * p;
    ExtractValue(y_for_value, &value_static);
    const Vector3<AutoDiffXd> y_for_gradient = R * p;
    ExtractGradient(y_for_gradient, {}, &gradient_static);
  });
  const int num_static_allocations_product = CountAllocations([&]() {
    ExtractValue(R * p, &value_static);
    ExtractGradient(R * p, {}, &gradient_static);
  });
  EXPECT_EQ(num_static_allocations_product, num_static_allocations_evaluated);
  EXPECT_TRUE(CompareMatrices(value_static, R * p_value, 1e-14));
  EXPECT_TRUE(CompareMatrices(gradient_static, R, 1e-14));
}

// Eigen returns each coefficient of a read-only view (e.g., Ref<const> or
// Map<const>) by value, which for AutoDiffXd copies its derivatives. Extracting
// from such views into preallocated outputs must not allocate.
GTEST_TEST(AdditionalAutodiffTest, ExtractFromReadOnlyViews) {
  MatrixX<AutoDiffXd> m(3, 4);
  for (int i = 0; i < m.size(); ++i) {
    m(i) = AutoDiffXd(i, Vector3d(i, 10 * i, 100 * i));
  }
  m(1, 2) = AutoDiffXd(7.0);  // No derivatives; extracted as zeros.
  const MatrixXd m_value = ExtractValue(m);
  const MatrixXd m_gradient = ExtractGradient(m);

  // A column, through Ref<const>.
  const Eigen::Ref<const VectorX<AutoDiffXd>> column = m.col(2);
  VectorXd column_value(3);
  MatrixXd column_gradient(3, 3);
  {
    test::LimitMalloc guard;
    ExtractValue(column, &column_value);
    ExtractGradient(column, {}, &column_gradient);
  }
  EXPECT_TRUE(CompareMatrices(column_value, m_value.col(2)));
  EXPECT_TRUE(CompareMatrices(column_gradient,
                              m_gradient.middleRows(2 * m.rows(), m.rows())));

  // A row, through a Map<const> with an inner stride.
  const Eigen::Map<const RowVectorX<AutoDiffXd>, 0, Eigen::InnerStride<>> row(
      m.data() + 1, m.cols(), Eigen::InnerStride<>(m.rows()));
  Eigen::RowVectorXd row_value(4);
  MatrixXd row_gradient(4, 3);
  {
    test::LimitMalloc guard;
    ExtractValue(row, &row_value);
    ExtractGradient(row, {}, &row_gradient);
  }
  EXPECT_TRUE(CompareMatrices(row_value, m_value.row(1)));
  for (int j = 0; j < m.cols(); ++j) {
    EXPECT_TRUE(
        CompareMatrices(row_gradient.row(j), m_gradient.row(1 + m.rows() * j)));
  }

  // The whole matrix, transposed, through Ref<const>. The transpose is
  // row-major, so its storage order matches that of m.
  const Eigen::Ref<const MatrixX<AutoDiffXd>> ref = m;
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>
      transpose_value(4, 3);
  MatrixXd transpose_gradient(12, 3);
  {
    test::LimitMalloc guard;
    ExtractValue(ref.transpose(), &transpose_value);
    ExtractGradient(ref.transpose(), {}, &transpose_gradient);
  }
  EXPECT_TRUE(CompareMatrices(transpose_value, m_value.transpose()));
  EXPECT_TRUE(CompareMatrices(transpose_gradient, m_gradient));

  // An expression without direct access, read by value.
  EXPECT_TRUE(CompareMatrices(ExtractValue(-m), -m_value));
  EXPECT_TRUE(CompareMatrices(ExtractGradient(-m), -m_gradient));

  // A row-major copy; the gradient rows follow its storage order.
  const Eigen::Matrix<AutoDiffXd, Eigen::Dynamic, Eigen::Dynamic,
                      Eigen::RowMajor>
      row_major = m;
  EXPECT_TRUE(CompareMatrices(ExtractValue(row_major), m_value));
  const MatrixXd row_major_gradient = ExtractGradient(row_major);
  for (int i = 0; i < m.rows(); ++i) {
    for (int j = 0; j < m.cols(); ++j) {
      EXPECT_TRUE(CompareMatrices(row_major_gradient.row(i * m.cols() + j),
                                  m_gradient.row(i + j * m.rows())));
    }
  }
}

// ExtractGradient() accepts expressions that can't be indexed by a single
// index, such as a non-vector Block or a Ref<const MatrixX>.
GTEST_TEST(AdditionalAutodiffTest, ExtractGradientWithoutLinearAccess) {
  MatrixX<AutoDiffXd> m(3, 4);
  for (int i = 0; i < m.size(); ++i) {
    m(i) = AutoDiffXd(i, Vector3d(i, 10 * i, 100 * i));
  }
  const Eigen::Ref<const MatrixX<AutoDiffXd>> ref = m;
  EXPECT_TRUE(CompareMatrices(ExtractGradient(ref), ExtractGradient(m)));
  const MatrixX<AutoDiffXd> block = m.block(1, 1, 2, 3);
  EXPECT_TRUE(CompareMatrices(ExtractGradient(m.block(1, 1, 2, 3)),
                              ExtractGradient(block)));
}

GTEST_TEST(AdditionalAutodiffTest, InitializeAutoDiffTuple) {
  // When all sizes are known at compile time, the resulting AutoDiffScalars
  // should have a compile time number of derivatives. Each matrix has a
  // distinct size, so that we can detect mistakes in the left-to-right vs
  // right-to-left order of derivative assignments.
  const Matrix2d matrix2 = Eigen::Matrix2d::Identity();
  const Vector3d vec3{1., 2., 3.};
  const Vector5d vec5 = VectorXd::LinSpaced(5, 5.0, 9.0);

  const auto tuple = math::InitializeAutoDiffTuple(matrix2, vec3, vec5);
  EXPECT_EQ(std::tuple_size_v<decltype(tuple)>, 3);

  EXPECT_EQ(std::get<0>(tuple).rows(), 2);
  EXPECT_EQ(std::get<0>(tuple).cols(), 2);
  EXPECT_EQ(std::get<1>(tuple).rows(), 3);
  EXPECT_EQ(std::get<1>(tuple).cols(), 1);
  EXPECT_EQ(std::get<2>(tuple).rows(), 5);
  EXPECT_EQ(std::get<2>(tuple).cols(), 1);

  // This is the expected type of the derivatives vector (in every element).
  const Eigen::Matrix<double, 12, 1>& deriv_12 =
      std::get<1>(tuple).coeffRef(2).derivatives();

  // Since vec3[2] is the 7th variable, we expect only element 7 of its
  // derivatives vector to be 1.
  VectorXd expected(12);
  expected << 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0;
  EXPECT_EQ(deriv_12, expected);

  // Now let's replace vec3 with a dynamic version of the same size.
  Eigen::VectorXd vec3d(3);
  vec3d << 10., 11., 12.;

  const auto tupled = math::InitializeAutoDiffTuple(matrix2, vec3d, vec5);
  EXPECT_EQ(std::tuple_size_v<decltype(tupled)>, 3);

  EXPECT_EQ(std::get<0>(tupled).rows(), 2);
  EXPECT_EQ(std::get<0>(tupled).cols(), 2);
  EXPECT_EQ(std::get<1>(tupled).rows(), 3);
  EXPECT_EQ(std::get<1>(tupled).cols(), 1);
  EXPECT_EQ(std::get<2>(tupled).rows(), 5);
  EXPECT_EQ(std::get<2>(tupled).cols(), 1);

  // This is the expected type of the derivatives vector (in every element).
  const Eigen::Matrix<double, Eigen::Dynamic, 1>& deriv_12d =
      std::get<1>(tupled).coeffRef(2).derivatives();

  // We should still get the same value at run time.
  expected << 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0;
  EXPECT_EQ(deriv_12d, expected);
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

GTEST_TEST(AutoDiffEqual, AreAutoDiffVecXdEqualTest) {
  VectorX<AutoDiffXd> a(2), b(2), c(3);
  a << 1, 2;
  b << 3, 4;
  c << 1, 2, 3;
  EXPECT_FALSE(AreAutoDiffVecXdEqual(a, b));
  EXPECT_FALSE(AreAutoDiffVecXdEqual(a, c));
  b = a;
  EXPECT_TRUE(AreAutoDiffVecXdEqual(a, b));
  a[0].derivatives() = Eigen::Vector2d(1, 2);
  b[0].derivatives() = Eigen::Vector3d(1, 2, 3);
  EXPECT_FALSE(AreAutoDiffVecXdEqual(a, b));
  b[0].derivatives() = Eigen::Vector2d(4, 5);
  EXPECT_FALSE(AreAutoDiffVecXdEqual(a, b));
  b[0].derivatives() = a[0].derivatives();
  EXPECT_TRUE(AreAutoDiffVecXdEqual(a, b));
  {
    test::LimitMalloc guard;
    EXPECT_TRUE(AreAutoDiffVecXdEqual(a, b));
  }
  // Set derivatives of b[0] and b[1] to have different sizes. ExtractGradient
  // will throw, but AreAutoDiffVecXdEqual should return false.
  b[1].derivatives() = Eigen::Vector3d(1, 2, 3);
  EXPECT_FALSE(AreAutoDiffVecXdEqual(a, b));
}

}  // namespace
}  // namespace math
}  // namespace drake
