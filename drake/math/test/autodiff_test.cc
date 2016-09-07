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

template <typename Derived>
void FillWithNumbersIncreasingFromZero(Eigen::MatrixBase<Derived>& matrix) {
  for (Eigen::Index i = 0; i < matrix.size(); i++) {
    matrix(i) = i;
  }
}

class AutodiffJacobianTest : public ::testing::Test {};

TEST_F(AutodiffJacobianTest, QuadraticForm) {
  using Eigen::Matrix3d;
  using Eigen::Vector3d;

  Matrix3d A;
  FillWithNumbersIncreasingFromZero(A);

  auto quadratic_form = [&](const auto& x) {
    using Scalar = typename std::remove_reference<decltype(x)>::type::Scalar;
    return (x.transpose() * A.cast<Scalar>().eval() * x).eval();
  };

  Vector3d x;
  FillWithNumbersIncreasingFromZero(x);
  auto jac_chunk_size_default = jacobian(quadratic_form, x);
  auto jac_chunk_size_1 = jacobian<1>(quadratic_form, x);
  auto jac_chunk_size_3 = jacobian<3>(quadratic_form, x);
  auto jac_chunk_size_6 = jacobian<6>(quadratic_form, x);

  // Ensure that chunk size has no effect on output type.
  static_assert(std::is_same<decltype(jac_chunk_size_default),
                             decltype(jac_chunk_size_1)>::value,
                "jacobian output type mismatch");
  static_assert(std::is_same<decltype(jac_chunk_size_default),
                             decltype(jac_chunk_size_3)>::value,
                "jacobian output type mismatch");
  static_assert(std::is_same<decltype(jac_chunk_size_default),
                             decltype(jac_chunk_size_6)>::value,
                "jacobian output type mismatch");

  // Ensure that the results are the same.
  EXPECT_TRUE(jac_chunk_size_default == jac_chunk_size_1);
  EXPECT_TRUE(jac_chunk_size_default == jac_chunk_size_3);
  EXPECT_TRUE(jac_chunk_size_default == jac_chunk_size_6);

  // Ensure that value is correct.
  auto value_expected = quadratic_form(x);
  auto value = autoDiffToValueMatrix(jac_chunk_size_default);
  EXPECT_TRUE(CompareMatrices(value_expected, value, 1e-12,
                              MatrixCompareType::absolute));

  // Ensure that Jacobian is correct.
  auto jac = autoDiffToGradientMatrix(jac_chunk_size_default);
  auto jac_expected = (x.transpose() * (A + A.transpose())).eval();
  EXPECT_TRUE(
      CompareMatrices(jac_expected, jac, 1e-12, MatrixCompareType::absolute));
}

class AutoDiffHessianTest : public ::testing::Test {};

// Example: quadratic function
// (A x + b)^T C (D x + e)
// from http://www.ee.ic.ac.uk/hp/staff/dmb/matrix/calculus.html#Hessian.
TEST_F(AutoDiffHessianTest, QuadraticFunction) {
  using Eigen::MatrixXd;
  using Eigen::VectorXd;
  using Eigen::Index;

  Index n = 4;
  Index m = 5;

  MatrixXd A(n, m);
  VectorXd b(n);
  MatrixXd C(n, n);
  MatrixXd D(n, m);
  VectorXd e(n);

  FillWithNumbersIncreasingFromZero(A);
  FillWithNumbersIncreasingFromZero(b);
  FillWithNumbersIncreasingFromZero(C);
  FillWithNumbersIncreasingFromZero(D);
  FillWithNumbersIncreasingFromZero(e);

  auto quadratic_function = [&](const auto& x) {
    using Scalar = typename std::remove_reference<decltype(x)>::type::Scalar;
    return ((A.cast<Scalar>() * x + b.cast<Scalar>()).transpose() *
            C.cast<Scalar>() * (D.cast<Scalar>() * x + e.cast<Scalar>()))
        .eval();
  };

  VectorXd x(m);
  FillWithNumbersIncreasingFromZero(x);

  auto hess_chunk_size_default = hessian(quadratic_function, x);
  auto hess_chunk_size_2_4 = hessian<2, 4>(quadratic_function, x);

  // Ensure that chunk size has no effect on output type.
  static_assert(std::is_same<decltype(hess_chunk_size_default),
                             decltype(hess_chunk_size_2_4)>::value,
                "hessian output type mismatch");

  // Ensure that the results are the same.
  EXPECT_TRUE(hess_chunk_size_default == hess_chunk_size_2_4);

  // Ensure that value is correct.
  auto value_expected = quadratic_function(x);
  auto value_autodiff = autoDiffToValueMatrix(hess_chunk_size_default);
  auto value = autoDiffToValueMatrix(value_autodiff);
  EXPECT_TRUE(CompareMatrices(value_expected, value, 1e-12,
                              MatrixCompareType::absolute));

  // Ensure that the two ways of computing the Jacobian from AutoDiff match.
  auto jac_autodiff = autoDiffToGradientMatrix(hess_chunk_size_default);
  auto jac1 = autoDiffToValueMatrix(jac_autodiff);
  auto jac2 = autoDiffToGradientMatrix(value_autodiff);
  EXPECT_TRUE(jac1 == jac2);

  // Ensure that the Jacobian is correct.
  auto jac_expected = ((A * x + b).transpose() * C * D +
                       (D * x + e).transpose() * C.transpose() * A)
                          .eval();
  EXPECT_TRUE(
      CompareMatrices(jac_expected, jac1, 1e-12, MatrixCompareType::absolute));

  // Ensure that the Hessian is correct.
  auto hess_expected =
      (A.transpose() * C * D + D.transpose() * C.transpose() * A).eval();
  auto hess = autoDiffToGradientMatrix(jac_autodiff);
  EXPECT_TRUE(
      CompareMatrices(hess_expected, hess, 1e-12, MatrixCompareType::absolute));
}

}  // namespace
}  // namespace math
}  // namespace drake
