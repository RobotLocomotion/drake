#include "drake/math/compute_numerical_gradient.h"

#include <limits>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/math/autodiff.h"
#include "drake/math/autodiff_gradient.h"

namespace drake {
namespace math {
namespace {

const double kEps = std::numeric_limits<double>::epsilon();

GTEST_TEST(ComputeNumericalGradientTest, TestAffineFunction) {
  // Compute the numerical gradient of an affine function y = A * x + b. The
  // numerical gradient be A.
  Eigen::Matrix<double, 3, 2> A;
  // Arbitrary A matrix.
  // clang-format off
  A << 1.2, 3,
       2.5, -4.25,
       1.5, 0;
  // clang-format on
  // Arbitrary b vector.
  Eigen::Vector3d b(1, 2, 3);
  // I need to create a std::function, instead of using a lambda directly, as
  // template deduction doesn't work when converting lambda to std::function,
  // explained in
  // https://stackoverflow.com/questions/26665152/compiler-does-not-deduce-template-parameters-map-stdvector-stdvector
  std::function<void(const Eigen::Vector2d& x, Eigen::Vector3d*)> calc_fun =
      [&A, &b](const Eigen::Matrix<double, 2, 1>& x,
               Eigen::Matrix<double, 3, 1>* y) -> void { *y = A * x + b; };

  auto check_gradient = [&A, &calc_fun](const Eigen::Vector2d& x) {
    NumericalGradientOption option;
    // forward difference
    option.method = NumericalGradientMethod::kForward;
    auto J = ComputeNumericalGradient(calc_fun, x, option);
    // The big tolerance is caused by the cancellation error in computing
    // f(x +  Δx) - f(x)), coming from the numerical roundoff error.
    const double tol = 1E-7;
    EXPECT_TRUE(CompareMatrices(J, A, tol));
    // backward difference
    option.method = NumericalGradientMethod::kBackward;
    J = ComputeNumericalGradient(calc_fun, x, option);
    EXPECT_TRUE(CompareMatrices(J, A, tol));
    // central difference
    option.method = NumericalGradientMethod::kCentral;
    J = ComputeNumericalGradient<Eigen::Vector2d, Eigen::Vector3d,
                                 Eigen::Vector2d>(calc_fun, x, option);
    EXPECT_TRUE(CompareMatrices(J, A, tol));
  };

  // Check gradient at different points.
  check_gradient(Eigen::Vector2d::Zero());
  check_gradient(Eigen::Vector2d(1, 2));
  check_gradient(Eigen::Vector2d(-1, 2E-10));
}

template <typename T>
void ToyFunction(const Vector3<T>& x, Vector2<T>* y) {
  using std::sin;
  (*y)(0) = x(0) + sin(x(1)) + x(0) * x(1) + 2;
  (*y)(1) = 2 * x(0) * x(2) + 3;
}

GTEST_TEST(ComputeNumericalGradientTest, TestToyFunction) {
  auto check_gradient = [](const Eigen::Vector3d& x) {
    NumericalGradientOption option;
    // forward difference.
    option.method = NumericalGradientMethod::kForward;
    std::function<void(const Eigen::Vector3d&, Eigen::Vector2d*)>
        ToyFunctionDouble = ToyFunction<double>;
    auto J = ComputeNumericalGradient(ToyFunctionDouble, x, option);
    auto x_autodiff = math::initializeAutoDiff<3>(x);
    Vector2<AutoDiffd<3>> y_autodiff;
    ToyFunction(x_autodiff, &y_autodiff);
    const double tol = 1E-7;
    EXPECT_TRUE(CompareMatrices(J, autoDiffToGradientMatrix(y_autodiff), tol));

    // backward difference
    option.method = NumericalGradientMethod::kBackward;
    J = ComputeNumericalGradient(ToyFunctionDouble, x, option);
    EXPECT_TRUE(CompareMatrices(J, autoDiffToGradientMatrix(y_autodiff), tol));

    // central difference
    option.method = NumericalGradientMethod::kCentral;
    J = ComputeNumericalGradient<Eigen::Vector3d, Eigen::Vector2d,
                                 Eigen::Vector3d>(ToyFunction<double>, x,
                                                  option);
    EXPECT_TRUE(CompareMatrices(J, autoDiffToGradientMatrix(y_autodiff), tol));
  };

  check_gradient(Eigen::Vector3d::Zero());
  check_gradient(Eigen::Vector3d(0, 1, 2));
  check_gradient(Eigen::Vector3d(0, -1, -2));
}
}  // namespace
}  // namespace math
}  // namespace drake
