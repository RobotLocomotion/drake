#include "drake/common/autodiff_overloads.h"

#include <type_traits>

#include <Eigen/Dense>
#include <gtest/gtest.h>
#include <unsupported/Eigen/AutoDiff>

#include "drake/common/cond.h"
#include "drake/common/eigen_matrix_compare.h"
#include "drake/common/eigen_types.h"
#include "drake/common/extract_double.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

namespace drake {
namespace common {
namespace {

// Tests ExtractDoubleOrThrow on autodiff.
GTEST_TEST(AutodiffOverloadsTest, ExtractDouble) {
  // On autodiff.
  Eigen::AutoDiffScalar<Eigen::Vector2d> x;
  x.value() = 1.0;
  EXPECT_EQ(ExtractDoubleOrThrow(x), 1.0);

  // A double still works, too.
  double y = 1.0;
  EXPECT_EQ(ExtractDoubleOrThrow(y), 1.0);
}

// Tests correctness of isinf.
GTEST_TEST(AutodiffOverloadsTest, IsInf) {
  Eigen::AutoDiffScalar<Eigen::Vector2d> x;
  x.value() = 1.0 / 0.0;
  EXPECT_EQ(isinf(x), true);
  x.value() = 0.0;
  EXPECT_EQ(isinf(x), false);
}

// Tests correctness of isnan.
GTEST_TEST(AutodiffOverloadsTest, IsNaN) {
  Eigen::AutoDiffScalar<Eigen::Vector2d> x;
  x.value() = 0.0 / 0.0;
  EXPECT_EQ(isnan(x), true);
  x.value() = 0.0;
  EXPECT_EQ(isnan(x), false);
}

GTEST_TEST(AutodiffOverloadsTests, CopySign) {
  Eigen::AutoDiffScalar<Eigen::Vector2d> x, y, z;
  x.derivatives() = Eigen::VectorXd::Unit(2, 0);
  y.derivatives() = Eigen::VectorXd::Unit(2, 1);

  // Positive, positive.
  x.value() = 1.1;
  y.value() = 2.5;
  z = copysign(x, y);
  EXPECT_DOUBLE_EQ(z.value(), x.value());
  EXPECT_TRUE(CompareMatrices(z.derivatives(), x.derivatives()));

  // Positive, negative.
  x.value() = 1.1;
  y.value() = -2.5;
  z = copysign(x, y);
  EXPECT_DOUBLE_EQ(z.value(), -x.value());
  EXPECT_TRUE(CompareMatrices(z.derivatives(), -x.derivatives()));

  // Negative, positive.
  x.value() = -1.1;
  y.value() = 2.5;
  z = copysign(x, y);
  EXPECT_DOUBLE_EQ(z.value(), -x.value());
  EXPECT_TRUE(CompareMatrices(z.derivatives(), -x.derivatives()));

  // Negative, negative.
  x.value() = -1.1;
  y.value() = -2.5;
  z = copysign(x, y);
  EXPECT_DOUBLE_EQ(z.value(), x.value());
  EXPECT_TRUE(CompareMatrices(z.derivatives(), x.derivatives()));

  // Test w/ double y (Negative, positive).
  z = copysign(x, 2.5);
  EXPECT_DOUBLE_EQ(z.value(), -x.value());
  EXPECT_TRUE(CompareMatrices(z.derivatives(), -x.derivatives()));
}

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

GTEST_TEST(AutodiffOverloadsTest, IfThenElse1) {
  using DerType = Eigen::Vector2d;
  Eigen::AutoDiffScalar<DerType> x{10.0, DerType{2, 1}};
  Eigen::AutoDiffScalar<DerType> y{if_then_else(x >= 0, x * x, x * x * x)};
  EXPECT_DOUBLE_EQ(y.value(), x.value() * x.value());
  EXPECT_DOUBLE_EQ(y.derivatives()[0], 2 * 2 * x.value());
  EXPECT_DOUBLE_EQ(y.derivatives()[1], 2 * x.value());
}

GTEST_TEST(AutodiffOverloadsTest, IfThenElse2) {
  using DerType = Eigen::Vector2d;
  Eigen::AutoDiffScalar<DerType> x{-10.0, DerType{2, 1}};
  Eigen::AutoDiffScalar<DerType> y{if_then_else(x >= 0, x * x, x * x * x)};
  EXPECT_DOUBLE_EQ(y.value(), x.value() * x.value() * x.value());
  EXPECT_DOUBLE_EQ(y.derivatives()[0], 2 * 3 * x.value() * x.value());
  EXPECT_DOUBLE_EQ(y.derivatives()[1], 3 * x.value() * x.value());
}

GTEST_TEST(AutodiffOverloadsTest, IfThenElse3) {
  using DerType = Eigen::Vector2d;
  Eigen::AutoDiffScalar<DerType> x{10.0, DerType{2, 1}};
  Eigen::AutoDiffScalar<DerType> y{if_then_else(x >= 0, x + 1, x)};
  EXPECT_DOUBLE_EQ(y.value(), x.value() + 1);
  EXPECT_DOUBLE_EQ(y.derivatives()[0], 2 * 1);
  EXPECT_DOUBLE_EQ(y.derivatives()[1], 1);
}

GTEST_TEST(AutodiffOverloadsTest, IfThenElse4) {
  using DerType = Eigen::Vector2d;
  Eigen::AutoDiffScalar<DerType> x{-10.0, DerType{2, 1}};
  Eigen::AutoDiffScalar<DerType> y{if_then_else(x >= 0, x, x)};
  EXPECT_DOUBLE_EQ(y.value(), x.value());
  EXPECT_DOUBLE_EQ(y.derivatives()[0], 2);
  EXPECT_DOUBLE_EQ(y.derivatives()[1], 1);
}

GTEST_TEST(AutodiffOverloadsTest, IfThenElse5) {
  using DerType = Eigen::Vector2d;
  Eigen::AutoDiffScalar<DerType> x{-10.0, DerType{2, 1}};
  Eigen::AutoDiffScalar<DerType> y{if_then_else(x >= 0, -x, -x * x * x)};
  EXPECT_DOUBLE_EQ(y.value(), -x.value() * x.value() * x.value());
  EXPECT_DOUBLE_EQ(y.derivatives()[0], 2 * -3 * x.value() * x.value());
  EXPECT_DOUBLE_EQ(y.derivatives()[1], -3 * x.value() * x.value());
}

GTEST_TEST(AutodiffOverloadsTest, IfThenElse6) {
  using DerType = Eigen::Vector2d;
  Eigen::AutoDiffScalar<DerType> x{10.0, DerType{2, 1}};
  Eigen::AutoDiffScalar<DerType> y{if_then_else(x >= 0, -x, x * x * x)};
  EXPECT_DOUBLE_EQ(y.value(), -x.value());
  EXPECT_DOUBLE_EQ(y.derivatives()[0], 2 * -1);
  EXPECT_DOUBLE_EQ(y.derivatives()[1], -1);
}

GTEST_TEST(AutodiffOverloadsTest, IfThenElse7) {
  using DerType = Eigen::Vector2d;
  Eigen::AutoDiffScalar<DerType> x{5.0, DerType{2, 1}};
  Eigen::AutoDiffScalar<DerType> y{10.0, DerType{2, 1}};
  Eigen::AutoDiffScalar<DerType> z{if_then_else(true, x * x, y * y * y)};
  EXPECT_DOUBLE_EQ(z.value(), x.value() * x.value());
  EXPECT_DOUBLE_EQ(z.derivatives()[0], 2 * 2 * x.value());
  EXPECT_DOUBLE_EQ(z.derivatives()[1], 2 * x.value());
}

GTEST_TEST(AutodiffOverloadsTest, IfThenElse8) {
  using DerType = Eigen::Vector2d;
  Eigen::AutoDiffScalar<DerType> x{5.0, DerType{2, 1}};
  Eigen::AutoDiffScalar<DerType> y{10.0, DerType{2, 1}};
  Eigen::AutoDiffScalar<DerType> z{if_then_else(false, x * x, y * y * y)};
  EXPECT_DOUBLE_EQ(z.value(), y.value() * y.value() * y.value());
  EXPECT_DOUBLE_EQ(z.derivatives()[0], 2 * 3 * y.value() * y.value());
  EXPECT_DOUBLE_EQ(z.derivatives()[1], 3 * y.value() * y.value());
}

GTEST_TEST(AutodiffOverloadsTest, Cond1) {
  using DerType = Eigen::Vector2d;
  Eigen::AutoDiffScalar<DerType> x{-10.0, DerType{2, 1}};
  Eigen::AutoDiffScalar<DerType> y{cond(x >= 0, x * x, x * x * x)};
  EXPECT_DOUBLE_EQ(y.value(), x.value() * x.value() * x.value());
  EXPECT_DOUBLE_EQ(y.derivatives()[0], 2 * 3 * x.value() * x.value());
  EXPECT_DOUBLE_EQ(y.derivatives()[1], 3 * x.value() * x.value());
}

GTEST_TEST(AutodiffOverloadsTest, Cond2) {
  using DerType = Eigen::Vector2d;
  Eigen::AutoDiffScalar<DerType> x{10.0, DerType{2, 1}};
  Eigen::AutoDiffScalar<DerType> y{cond(x >= 0, x * x, x * x * x)};
  EXPECT_DOUBLE_EQ(y.value(), x.value() * x.value());
  EXPECT_DOUBLE_EQ(y.derivatives()[0], 2 * 2 * x.value());
  EXPECT_DOUBLE_EQ(y.derivatives()[1], 2 * x.value());
}

GTEST_TEST(AutodiffOverloadsTest, Cond3) {
  using DerType = Eigen::Vector2d;
  Eigen::AutoDiffScalar<DerType> x{10.0, DerType{2, 1}};
  // clang-format off
  Eigen::AutoDiffScalar<DerType> y{cond(x >= 10, 10 * x * x,
                                        x >= 5, 5 * x * x * x,
                                        x >= 3, -3 * x,
                                        x * x)};
  // clang-format on
  EXPECT_DOUBLE_EQ(y.value(), 10 * x.value() * x.value());
  EXPECT_DOUBLE_EQ(y.derivatives()[0], 2 * 10 * 2 * x.value());
  EXPECT_DOUBLE_EQ(y.derivatives()[1], 10 * 2 * x.value());
}

GTEST_TEST(AutodiffOverloadsTest, Cond4) {
  using DerType = Eigen::Vector2d;
  Eigen::AutoDiffScalar<DerType> x{8.0, DerType{2, 1}};
  // clang-format off
  Eigen::AutoDiffScalar<DerType> y{cond(x >= 10, 10 * x * x,
                                        x >= 5, 5 * x * x * x,
                                        x >= 3, -3 * x,
                                        x * x)};
  // clang-format on
  EXPECT_DOUBLE_EQ(y.value(), 5 * x.value() * x.value() * x.value());
  EXPECT_DOUBLE_EQ(y.derivatives()[0], 2 * 5 * 3 * x.value() * x.value());
  EXPECT_DOUBLE_EQ(y.derivatives()[1], 5 * 3 * x.value() * x.value());
}

GTEST_TEST(AutodiffOverloadsTest, Cond5) {
  using DerType = Eigen::Vector2d;
  Eigen::AutoDiffScalar<DerType> x{4.0, DerType{2, 1}};
  // clang-format off
  Eigen::AutoDiffScalar<DerType> y{cond(x >= 10, 10 * x * x,
                                        x >= 5, 5 * x * x * x,
                                        x >= 3, -3 * x,
                                        x * x)};
  // clang-format on
  EXPECT_DOUBLE_EQ(y.value(), -3 * x.value());
  EXPECT_DOUBLE_EQ(y.derivatives()[0], 2 * -3);
  EXPECT_DOUBLE_EQ(y.derivatives()[1], -3);
}

GTEST_TEST(AutodiffOverloadsTest, Cond6) {
  using DerType = Eigen::Vector2d;
  Eigen::AutoDiffScalar<DerType> x{2.0, DerType{2, 1}};
  // clang-format off
  Eigen::AutoDiffScalar<DerType> y{cond(x >= 10, 10 * x * x,
                                        x >= 5, 5 * x * x * x,
                                        x >= 3, -3 * x,
                                        x * x)};
  // clang-format on
  EXPECT_DOUBLE_EQ(y.value(), x.value() * x.value());
  EXPECT_DOUBLE_EQ(y.derivatives()[0], 2 * 2 * x.value());
  EXPECT_DOUBLE_EQ(y.derivatives()[1], 2 * x.value());
}

GTEST_TEST(AutodiffOverloadsTest, Cond7) {
  using DerType = Eigen::Vector2d;
  Eigen::AutoDiffScalar<DerType> x{10.0, DerType{2, 1}};
  Eigen::AutoDiffScalar<DerType> y{2.0, DerType{4, 2}};
  // clang-format off
  Eigen::AutoDiffScalar<DerType> z{cond(x >= 10, 10 * x * x,
                                        x >= 5, 5 * y * y * y,
                                        x * x)};
  // clang-format on
  EXPECT_DOUBLE_EQ(z.value(), 10 * x.value() * x.value());
  EXPECT_DOUBLE_EQ(z.derivatives()[0], 2 * 10 * 2 * x.value());
  EXPECT_DOUBLE_EQ(z.derivatives()[1], 10 * 2 * x.value());
}

GTEST_TEST(AutodiffOverloadsTest, Cond8) {
  using DerType = Eigen::Vector2d;
  Eigen::AutoDiffScalar<DerType> x{7.0, DerType{2, 1}};
  Eigen::AutoDiffScalar<DerType> y{2.0, DerType{4, 2}};
  // clang-format off
  Eigen::AutoDiffScalar<DerType> z{cond(x >= 10, 10 * x * x,
                                        x >= 5, 5 * y * y * y,
                                        x * x)};
  // clang-format on
  EXPECT_DOUBLE_EQ(z.value(), 5 * y.value() * y.value() * y.value());
  EXPECT_DOUBLE_EQ(z.derivatives()[0], 4 * 5 * 3 * y.value() * y.value());
  EXPECT_DOUBLE_EQ(z.derivatives()[1], 2 * 5 * 3 * y.value() * y.value());
}

GTEST_TEST(AutodiffOverloadsTest, Cond9) {
  using DerType = Eigen::Vector2d;
  Eigen::AutoDiffScalar<DerType> x{3, DerType{2, 1}};
  Eigen::AutoDiffScalar<DerType> y{2.0, DerType{4, 2}};
  // clang-format off
  Eigen::AutoDiffScalar<DerType> z{cond(x >= 10, 10 * x * x,
                                        x >= 5, 5 * y * y * y,
                                        x * x)};
  // clang-format on
  EXPECT_DOUBLE_EQ(z.value(), x.value() * x.value());
  EXPECT_DOUBLE_EQ(z.derivatives()[0], 2 * 2 * x.value());
  EXPECT_DOUBLE_EQ(z.derivatives()[1], 2 * x.value());
}

// This is just a sanity check to make sure that Eigen::NumTraits::Literal
// is the right way to dig through an AutoDiffScalar to find the underlying
// floating point type. If this compiles it succeeds.
GTEST_TEST(AutodiffOverloadsTest, CheckEigenLiteral) {
  using DerTyped = Eigen::Vector2d;
  using DerTypef = Eigen::Vector2f;
  using Td = Eigen::AutoDiffScalar<DerTyped>;
  using Tf = Eigen::AutoDiffScalar<DerTypef>;

  using Literald = typename Eigen::NumTraits<Td>::Literal;
  using Literalf = typename Eigen::NumTraits<Tf>::Literal;

  static_assert(std::is_same<Literald, double>::value &&
                    std::is_same<Literalf, float>::value,
                "Eigen::NumTraits<T>::Literal didn't behave as expected.");
}

GTEST_TEST(AutodiffOverloadsTest, DummyValueX) {
  using T = Eigen::AutoDiffScalar<Eigen::VectorXd>;
  const T dummy_xd = dummy_value<T>::get();
  const double value = dummy_xd.value();
  EXPECT_TRUE(std::isnan(value));
  const Eigen::VectorXd derivatives = dummy_xd.derivatives();
  EXPECT_EQ(derivatives.rows(), 0);
}

GTEST_TEST(AutodiffOverloadsTest, DummyValue2) {
  using T = Eigen::AutoDiffScalar<Eigen::Vector2d>;
  const T dummy_2d = dummy_value<T>::get();
  const double value = dummy_2d.value();
  EXPECT_TRUE(std::isnan(value));
  const Eigen::Vector2d derivatives = dummy_2d.derivatives();
  EXPECT_EQ(derivatives.rows(), 2);
  EXPECT_TRUE(std::isnan(derivatives(0)));
  EXPECT_TRUE(std::isnan(derivatives(1)));
}

}  // namespace
}  // namespace common
}  // namespace drake
