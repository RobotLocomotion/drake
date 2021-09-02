#include "drake/math/jacobian.h"

#include <Eigen/Dense>
#include <gtest/gtest.h>

#include "drake/common/autodiff.h"
#include "drake/common/eigen_types.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/math/autodiff_gradient.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

namespace drake {
namespace math {
namespace {

template <typename Derived>
// TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
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

  // Work around GCC 5.4 Wshadow bug; the bug is fixed as of GCC 6.1.
  // https://gcc.gnu.org/bugzilla/show_bug.cgi?id=67273.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wshadow"
  auto quadratic_form = [&](const auto& x) {
#pragma GCC diagnostic pop
    using Scalar = typename std::remove_reference_t<decltype(x)>::Scalar;
    return (x.transpose() * A.cast<Scalar>().eval() * x).eval();
  };

  Vector3d x;
  FillWithNumbersIncreasingFromZero(x);
  auto jac_chunk_size_default = jacobian(quadratic_form, x);
  auto jac_chunk_size_1 = jacobian<1>(quadratic_form, x);
  auto jac_chunk_size_3 = jacobian<3>(quadratic_form, x);
  auto jac_chunk_size_6 = jacobian<6>(quadratic_form, x);

  // Ensure that chunk size has no effect on output type.
  static_assert(std::is_same_v<decltype(jac_chunk_size_default),
                               decltype(jac_chunk_size_1)>,
                "jacobian output type mismatch");
  static_assert(std::is_same_v<decltype(jac_chunk_size_default),
                               decltype(jac_chunk_size_3)>,
                "jacobian output type mismatch");
  static_assert(std::is_same_v<decltype(jac_chunk_size_default),
                               decltype(jac_chunk_size_6)>,
                "jacobian output type mismatch");

  // Ensure that the results are the same.
  EXPECT_TRUE(jac_chunk_size_default == jac_chunk_size_1);
  EXPECT_TRUE(jac_chunk_size_default == jac_chunk_size_3);
  EXPECT_TRUE(jac_chunk_size_default == jac_chunk_size_6);

  // Ensure that value is correct.
  auto value_expected = quadratic_form(x);
  auto value = ExtractValue(jac_chunk_size_default);
  EXPECT_TRUE(CompareMatrices(value_expected, value, 1e-12,
                              MatrixCompareType::absolute));

  // Ensure that Jacobian is correct.
  auto jac = ExtractGradient(jac_chunk_size_default);
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

  // Work around GCC 5.4 Wshadow bug; the bug is fixed as of GCC 6.1.
  // https://gcc.gnu.org/bugzilla/show_bug.cgi?id=67273.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wshadow"
  auto quadratic_function = [&](const auto& x) {
#pragma GCC diagnostic pop
    using Scalar = typename std::remove_reference_t<decltype(x)>::Scalar;
    return ((A.cast<Scalar>() * x + b.cast<Scalar>()).transpose() *
            C.cast<Scalar>() * (D.cast<Scalar>() * x + e.cast<Scalar>()))
        .eval();
  };

  VectorXd x(m);
  FillWithNumbersIncreasingFromZero(x);

  auto hess_chunk_size_default = hessian(quadratic_function, x);
  auto hess_chunk_size_2_4 = hessian<2, 4>(quadratic_function, x);

  // Ensure that chunk size has no effect on output type.
  static_assert(std::is_same_v<decltype(hess_chunk_size_default),
                               decltype(hess_chunk_size_2_4)>,
                "hessian output type mismatch");

  // Ensure that the results are the same.
  EXPECT_TRUE(hess_chunk_size_default == hess_chunk_size_2_4);

  // Ensure that value is correct.
  auto value_expected = quadratic_function(x);
  auto value_autodiff = ExtractValue(hess_chunk_size_default);
  auto value = ExtractValue(value_autodiff);
  EXPECT_TRUE(CompareMatrices(value_expected, value, 1e-12,
                              MatrixCompareType::absolute));

  // Ensure that the two ways of computing the Jacobian from AutoDiff match.
  auto jac_autodiff = ExtractGradient(hess_chunk_size_default);
  auto jac1 = ExtractValue(jac_autodiff);
  auto jac2 = ExtractGradient(value_autodiff);
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
  auto hess = ExtractGradient(jac_autodiff);
  EXPECT_TRUE(
      CompareMatrices(hess_expected, hess, 1e-12, MatrixCompareType::absolute));
}

}  // namespace
}  // namespace math
}  // namespace drake
