#include "drake/math/autodiff_overloads.h"

#include <Eigen/Dense>
#include <unsupported/Eigen/AutoDiff>

#include "gtest/gtest.h"

#include "drake/common/eigen_types.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

namespace drake {
namespace math {
namespace {

// Tests that pow(AutoDiffScalar, AutoDiffScalar) applies the chain rule.
GTEST_TEST(AutodiffOverloadsTest, Pow) {
  Eigen::AutoDiffScalar<Eigen::Vector2d> x;
  x.value() = 1.1;
  x.derivatives() = Eigen::VectorXd::Unit(2, 0);
  Eigen::AutoDiffScalar<Eigen::Vector2d> y;
  y.value() = 2.5;
  y.derivatives() = Eigen::VectorXd::Unit(2, 1);

  x = x * (y + 2);
  EXPECT_DOUBLE_EQ(4.95, x.value());

  // The derivative of x with respect to its original value is y + 2 = 4.5.
  EXPECT_DOUBLE_EQ(4.5, x.derivatives()[0]);
  // The derivative of x with respect to y is x = 1.1.
  EXPECT_DOUBLE_EQ(1.1, x.derivatives()[1]);

  auto z = pow(x, y);
  // z is x^y = 4.95^2.5 ~= 54.51.
  EXPECT_DOUBLE_EQ(std::pow(4.95, 2.5), z.value());
  // ∂z/∂x is y*x^(y-1) = 2.5 * 4.95^1.5 ~= 27.53.
  const double dzdx = 2.5 * std::pow(4.95, 1.5);
  // ∂z/∂y is (x^y)*ln(x) = (4.95^2.5)*ln(4.95) ~= 87.19.
  const double dzdy = std::pow(4.95, 2.5) * std::log(4.95);
  // By the chain rule, dz/dv is 27.53 * xgrad + 87.19 * ygrad
  EXPECT_DOUBLE_EQ(dzdx * 4.5 + dzdy * 0.0, z.derivatives()[0]);
  EXPECT_DOUBLE_EQ(dzdx * 1.1 + dzdy * 1.0, z.derivatives()[1]);
}

}  // namespace
}  // namespace math
}  // namespace drake
