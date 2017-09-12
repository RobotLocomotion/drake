#include "drake/common/autodiff_overloads.h"

#include <type_traits>

#include <Eigen/Dense>
#include <gtest/gtest.h>

#include "drake/common/cond.h"
#include "drake/common/eigen_autodiff_types.h"
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

class AutoDiffXdTest : public ::testing::Test {
 protected:
  void SetUp() override {
    // We only set the derivatives of y and leave x's uninitialized.
    y_xd_.derivatives() = Eigen::VectorXd::Ones(3);
    y_3d_.derivatives() = Eigen::VectorXd::Ones(3);
  }

  // Evaluates a given function f with values of AutoDiffXd and values with
  // AutoDiffd<3>. It checks if the values and the derivatives of those
  // evaluation results are matched.
  template <typename F>
  ::testing::AssertionResult Check(const F& f) {
    const AutoDiffXd e1{f(x_xd_, y_xd_)};
    const AutoDiffd<3> e2{f(x_3d_, y_3d_)};
    if (e1.value() != e2.value()) {
      return ::testing::AssertionFailure()
             << "Values do not match: " << e1.value() << " and " << e2.value();
    }
    if (e1.derivatives() != e2.derivatives()) {
      return ::testing::AssertionFailure() << "Derivatives do not match:\n"
                                           << e1.derivatives() << "\n----\n"
                                           << e2.derivatives() << "\n";
    }
    return ::testing::AssertionSuccess();
  }

  // AutoDiffXd constants -- x and y.
  const AutoDiffXd x_xd_{0.5};
  AutoDiffXd y_xd_{0.7};
  // AutoDiffd<3> constants -- x and y.
  const AutoDiffd<3> x_3d_{x_xd_.value()};
  AutoDiffd<3> y_3d_{y_xd_.value()};
};

// We need to specify the return type of the polymorphic lambda function that is
// passed to AutoDiffXdTest::Check() method.
#define DRAKE_NO_REF_ADS_TYPE(x) \
  typename Eigen::internal::remove_reference<decltype(x)>::type

TEST_F(AutoDiffXdTest, Addition) {
  EXPECT_TRUE(
      Check([](const auto& x, const auto& y) -> DRAKE_NO_REF_ADS_TYPE(x) {
        return (x + x) + (y + x);
      }));
  EXPECT_TRUE(
      Check([](const auto& x, const auto& y) -> DRAKE_NO_REF_ADS_TYPE(x) {
        return (y + y) + (x + y);
      }));
  EXPECT_TRUE(
      Check([](const auto& x, const auto& y) -> DRAKE_NO_REF_ADS_TYPE(x) {
        return x + 3.0 + y;
      }));
  EXPECT_TRUE(
      Check([](const auto& x, const auto& y) -> DRAKE_NO_REF_ADS_TYPE(x) {
        return 3.0 + x + y;
      }));
  EXPECT_TRUE(
      Check([](const auto& x, const auto& y) -> DRAKE_NO_REF_ADS_TYPE(x) {
        return x + (y + 3.0);
      }));
  EXPECT_TRUE(
      Check([](const auto& x, const auto& y) -> DRAKE_NO_REF_ADS_TYPE(x) {
        return x + (3.0 + y);
      }));
  EXPECT_TRUE(
      Check([](const auto& x, const auto& y) -> DRAKE_NO_REF_ADS_TYPE(x) {
        return 3.0 + (y + x);
      }));
  EXPECT_TRUE(
      Check([](const auto& x, const auto& y) -> DRAKE_NO_REF_ADS_TYPE(x) {
        return 3.0 + (x + y);
      }));
}

TEST_F(AutoDiffXdTest, Subtraction) {
  EXPECT_TRUE(
      Check([](const auto& x, const auto& y) -> DRAKE_NO_REF_ADS_TYPE(x) {
        return (x + x) - (y - x);
      }));
  EXPECT_TRUE(
      Check([](const auto& x, const auto& y) -> DRAKE_NO_REF_ADS_TYPE(x) {
        return (y + y) - (x - y);
      }));
  EXPECT_TRUE(
      Check([](const auto& x, const auto& y) -> DRAKE_NO_REF_ADS_TYPE(x) {
        return x - 3.0 - y;
      }));
  EXPECT_TRUE(
      Check([](const auto& x, const auto& y) -> DRAKE_NO_REF_ADS_TYPE(x) {
        return 3.0 - x - y;
      }));
  EXPECT_TRUE(
      Check([](const auto& x, const auto& y) -> DRAKE_NO_REF_ADS_TYPE(x) {
        return x - (y - 3.0);
      }));
  EXPECT_TRUE(
      Check([](const auto& x, const auto& y) -> DRAKE_NO_REF_ADS_TYPE(x) {
        return x - (3.0 - y);
      }));
  EXPECT_TRUE(
      Check([](const auto& x, const auto& y) -> DRAKE_NO_REF_ADS_TYPE(x) {
        return 3.0 - (y - x);
      }));
  EXPECT_TRUE(
      Check([](const auto& x, const auto& y) -> DRAKE_NO_REF_ADS_TYPE(x) {
        return 3.0 - (x - y);
      }));
}

TEST_F(AutoDiffXdTest, Multiplication) {
  EXPECT_TRUE(
      Check([](const auto& x, const auto& y) -> DRAKE_NO_REF_ADS_TYPE(x) {
        return (x + x) * (y * x);
      }));
  EXPECT_TRUE(
      Check([](const auto& x, const auto& y) -> DRAKE_NO_REF_ADS_TYPE(x) {
        return (y + y) * (x * y);
      }));
  EXPECT_TRUE(
      Check([](const auto& x, const auto& y) -> DRAKE_NO_REF_ADS_TYPE(x) {
        return x * 3.0 * y;
      }));
  EXPECT_TRUE(
      Check([](const auto& x, const auto& y) -> DRAKE_NO_REF_ADS_TYPE(x) {
        return 3.0 * x * y;
      }));
  EXPECT_TRUE(
      Check([](const auto& x, const auto& y) -> DRAKE_NO_REF_ADS_TYPE(x) {
        return x * (y * 3.0);
      }));
  EXPECT_TRUE(
      Check([](const auto& x, const auto& y) -> DRAKE_NO_REF_ADS_TYPE(x) {
        return x * (3.0 * y);
      }));
  EXPECT_TRUE(
      Check([](const auto& x, const auto& y) -> DRAKE_NO_REF_ADS_TYPE(x) {
        return 3.0 * (y * x);
      }));
  EXPECT_TRUE(
      Check([](const auto& x, const auto& y) -> DRAKE_NO_REF_ADS_TYPE(x) {
        return 3.0 * (x * y);
      }));
}

TEST_F(AutoDiffXdTest, Division) {
  EXPECT_TRUE(
      Check([](const auto& x, const auto& y) -> DRAKE_NO_REF_ADS_TYPE(x) {
        return (x + x) / (y * x);
      }));
  EXPECT_TRUE(
      Check([](const auto& x, const auto& y) -> DRAKE_NO_REF_ADS_TYPE(x) {
        return (y + y) / (x * y);
      }));
  EXPECT_TRUE(
      Check([](const auto& x, const auto& y) -> DRAKE_NO_REF_ADS_TYPE(x) {
        return x / 3.0 / y;
      }));
  EXPECT_TRUE(
      Check([](const auto& x, const auto& y) -> DRAKE_NO_REF_ADS_TYPE(x) {
        return 3.0 / x / y;
      }));
  EXPECT_TRUE(
      Check([](const auto& x, const auto& y) -> DRAKE_NO_REF_ADS_TYPE(x) {
        return x / (y / 3.0);
      }));
  EXPECT_TRUE(
      Check([](const auto& x, const auto& y) -> DRAKE_NO_REF_ADS_TYPE(x) {
        return x / (3.0 / y);
      }));
  EXPECT_TRUE(
      Check([](const auto& x, const auto& y) -> DRAKE_NO_REF_ADS_TYPE(x) {
        return 3.0 / (y / x);
      }));
  EXPECT_TRUE(
      Check([](const auto& x, const auto& y) -> DRAKE_NO_REF_ADS_TYPE(x) {
        return 3.0 / (x / y);
      }));
}

TEST_F(AutoDiffXdTest, Pow) {
  EXPECT_TRUE(
      Check([](const auto& x, const auto& y) -> DRAKE_NO_REF_ADS_TYPE(x) {
        return pow(x + x, y + y);
      }));
  EXPECT_TRUE(
      Check([](const auto& x, const auto& y) -> DRAKE_NO_REF_ADS_TYPE(x) {
        return pow(x + 0.1, y + y);
      }));
  EXPECT_TRUE(
      Check([](const auto& x, const auto& y) -> DRAKE_NO_REF_ADS_TYPE(x) {
        return pow(y + y, x + x);
      }));
  EXPECT_TRUE(
      Check([](const auto& x, const auto& y) -> DRAKE_NO_REF_ADS_TYPE(x) {
        return pow(y + 0.1, x + x);
      }));
  EXPECT_TRUE(
      Check([](const auto& x, const auto& y) -> DRAKE_NO_REF_ADS_TYPE(x) {
        return pow(x + x, 2.0) * (y * y);
      }));
  EXPECT_TRUE(
      Check([](const auto& x, const auto& y) -> DRAKE_NO_REF_ADS_TYPE(x) {
        return pow(x + 0.1, 2.0) * (y * y);
      }));
  EXPECT_TRUE(
      Check([](const auto& x, const auto& y) -> DRAKE_NO_REF_ADS_TYPE(x) {
        return pow(y + y, 2.0) * (x * x);
      }));
  EXPECT_TRUE(
      Check([](const auto& x, const auto& y) -> DRAKE_NO_REF_ADS_TYPE(x) {
        return pow(y + 0.1, 2.0) * (x * x);
      }));
  // Note that Eigen's AutoDiffScalar.h does not provide an implementation for
  // pow(double, ADS). Therefore, we do not provide its specialization for
  // AutoDiffScalar here either.
}

TEST_F(AutoDiffXdTest, Atan2) {
  EXPECT_TRUE(
      Check([](const auto& x, const auto& y) -> DRAKE_NO_REF_ADS_TYPE(x) {
        return atan2(x + x, y + y);
      }));
  EXPECT_TRUE(
      Check([](const auto& x, const auto& y) -> DRAKE_NO_REF_ADS_TYPE(x) {
        return atan2(x + 0.1, y + y);
      }));
  EXPECT_TRUE(
      Check([](const auto& x, const auto& y) -> DRAKE_NO_REF_ADS_TYPE(x) {
        return atan2(y + y, x + x);
      }));
  EXPECT_TRUE(
      Check([](const auto& x, const auto& y) -> DRAKE_NO_REF_ADS_TYPE(x) {
        return atan2(y + 0.1, x + x);
      }));
}

TEST_F(AutoDiffXdTest, Abs) {
  EXPECT_TRUE(
      Check([](const auto& x, const auto& y) -> DRAKE_NO_REF_ADS_TYPE(x) {
        return abs(x + x) + y;
      }));
  EXPECT_TRUE(
      Check([](const auto& x, const auto& y) -> DRAKE_NO_REF_ADS_TYPE(x) {
        return abs(x + 0.1) + y;
      }));
  EXPECT_TRUE(
      Check([](const auto& x, const auto& y) -> DRAKE_NO_REF_ADS_TYPE(x) {
        return abs(y + y) + x;
      }));
  EXPECT_TRUE(
      Check([](const auto& x, const auto& y) -> DRAKE_NO_REF_ADS_TYPE(x) {
        return abs(y + 0.1) + x;
      }));
}

TEST_F(AutoDiffXdTest, Abs2) {
  EXPECT_TRUE(
      Check([](const auto& x, const auto& y) -> DRAKE_NO_REF_ADS_TYPE(x) {
        return abs2(x - x) + y;
      }));
  EXPECT_TRUE(
      Check([](const auto& x, const auto& y) -> DRAKE_NO_REF_ADS_TYPE(x) {
        return abs2(x - 0.1) + y;
      }));
  EXPECT_TRUE(
      Check([](const auto& x, const auto& y) -> DRAKE_NO_REF_ADS_TYPE(x) {
        return abs2(y - y) + x;
      }));
  EXPECT_TRUE(
      Check([](const auto& x, const auto& y) -> DRAKE_NO_REF_ADS_TYPE(x) {
        return abs2(y - 0.1) + x;
      }));
}

TEST_F(AutoDiffXdTest, Sqrt) {
  EXPECT_TRUE(
      Check([](const auto& x, const auto& y) -> DRAKE_NO_REF_ADS_TYPE(x) {
        return sqrt(x * x) + y;
      }));
  EXPECT_TRUE(
      Check([](const auto& x, const auto& y) -> DRAKE_NO_REF_ADS_TYPE(x) {
        return sqrt(0.1 * x) + y;
      }));
  EXPECT_TRUE(
      Check([](const auto& x, const auto& y) -> DRAKE_NO_REF_ADS_TYPE(x) {
        return sqrt(y * y) + x;
      }));
  EXPECT_TRUE(
      Check([](const auto& x, const auto& y) -> DRAKE_NO_REF_ADS_TYPE(x) {
        return sqrt(0.1 * y) + x;
      }));
}

TEST_F(AutoDiffXdTest, Cos) {
  EXPECT_TRUE(
      Check([](const auto& x, const auto& y) -> DRAKE_NO_REF_ADS_TYPE(x) {
        return cos(x + x) + y;
      }));
  EXPECT_TRUE(
      Check([](const auto& x, const auto& y) -> DRAKE_NO_REF_ADS_TYPE(x) {
        return cos(-(x + 0.1)) + y;
      }));
  EXPECT_TRUE(
      Check([](const auto& x, const auto& y) -> DRAKE_NO_REF_ADS_TYPE(x) {
        return cos(y + y) + x;
      }));
  EXPECT_TRUE(
      Check([](const auto& x, const auto& y) -> DRAKE_NO_REF_ADS_TYPE(x) {
        return cos(y + 0.1) + x;
      }));
}

TEST_F(AutoDiffXdTest, Sin) {
  EXPECT_TRUE(
      Check([](const auto& x, const auto& y) -> DRAKE_NO_REF_ADS_TYPE(x) {
        return sin(x - x) + y;
      }));
  EXPECT_TRUE(
      Check([](const auto& x, const auto& y) -> DRAKE_NO_REF_ADS_TYPE(x) {
        return sin(x - 0.1) + y;
      }));
  EXPECT_TRUE(
      Check([](const auto& x, const auto& y) -> DRAKE_NO_REF_ADS_TYPE(x) {
        return sin(y - y) + x;
      }));
  EXPECT_TRUE(
      Check([](const auto& x, const auto& y) -> DRAKE_NO_REF_ADS_TYPE(x) {
        return sin(y - 0.1) + x;
      }));
}

TEST_F(AutoDiffXdTest, Exp) {
  EXPECT_TRUE(
      Check([](const auto& x, const auto& y) -> DRAKE_NO_REF_ADS_TYPE(x) {
        return exp(x * x) + y;
      }));
  EXPECT_TRUE(
      Check([](const auto& x, const auto& y) -> DRAKE_NO_REF_ADS_TYPE(x) {
        return exp(x * 0.1) + y;
      }));
  EXPECT_TRUE(
      Check([](const auto& x, const auto& y) -> DRAKE_NO_REF_ADS_TYPE(x) {
        return exp(y * y) + x;
      }));
  EXPECT_TRUE(
      Check([](const auto& x, const auto& y) -> DRAKE_NO_REF_ADS_TYPE(x) {
        return exp(y * 0.1) + x;
      }));
}

TEST_F(AutoDiffXdTest, Log) {
  EXPECT_TRUE(
      Check([](const auto& x, const auto& y) -> DRAKE_NO_REF_ADS_TYPE(x) {
        return log(x / x) + y;
      }));
  EXPECT_TRUE(
      Check([](const auto& x, const auto& y) -> DRAKE_NO_REF_ADS_TYPE(x) {
        return log(x / 0.1) + y;
      }));
  EXPECT_TRUE(
      Check([](const auto& x, const auto& y) -> DRAKE_NO_REF_ADS_TYPE(x) {
        return log(y / y) + x;
      }));
  EXPECT_TRUE(
      Check([](const auto& x, const auto& y) -> DRAKE_NO_REF_ADS_TYPE(x) {
        return log(y / 0.1) + x;
      }));
}

TEST_F(AutoDiffXdTest, Tan) {
  EXPECT_TRUE(
      Check([](const auto& x, const auto& y) -> DRAKE_NO_REF_ADS_TYPE(x) {
        return tan(x + x) + y;
      }));
  EXPECT_TRUE(
      Check([](const auto& x, const auto& y) -> DRAKE_NO_REF_ADS_TYPE(x) {
        return tan(0.1 + x) + y;
      }));
  EXPECT_TRUE(
      Check([](const auto& x, const auto& y) -> DRAKE_NO_REF_ADS_TYPE(x) {
        return tan(y + y) + x;
      }));
  EXPECT_TRUE(
      Check([](const auto& x, const auto& y) -> DRAKE_NO_REF_ADS_TYPE(x) {
        return tan(0.1 + y) + x;
      }));
}

TEST_F(AutoDiffXdTest, Asin) {
  EXPECT_TRUE(
      Check([](const auto& x, const auto& y) -> DRAKE_NO_REF_ADS_TYPE(x) {
        return asin(x * x) + y;
      }));
  EXPECT_TRUE(
      Check([](const auto& x, const auto& y) -> DRAKE_NO_REF_ADS_TYPE(x) {
        return asin(x * 0.1) + y;
      }));
  EXPECT_TRUE(
      Check([](const auto& x, const auto& y) -> DRAKE_NO_REF_ADS_TYPE(x) {
        return asin(y * y) + x;
      }));
  EXPECT_TRUE(
      Check([](const auto& x, const auto& y) -> DRAKE_NO_REF_ADS_TYPE(x) {
        return asin(y * 0.1) + x;
      }));
}

TEST_F(AutoDiffXdTest, Acos) {
  EXPECT_TRUE(
      Check([](const auto& x, const auto& y) -> DRAKE_NO_REF_ADS_TYPE(x) {
        return acos(x * x) + y;
      }));
  EXPECT_TRUE(
      Check([](const auto& x, const auto& y) -> DRAKE_NO_REF_ADS_TYPE(x) {
        return acos(0.1 - x) + y;
      }));
  EXPECT_TRUE(
      Check([](const auto& x, const auto& y) -> DRAKE_NO_REF_ADS_TYPE(x) {
        return acos(-x) + y;
      }));

  EXPECT_TRUE(
      Check([](const auto& x, const auto& y) -> DRAKE_NO_REF_ADS_TYPE(x) {
        return acos(y * y) + x;
      }));
  EXPECT_TRUE(
      Check([](const auto& x, const auto& y) -> DRAKE_NO_REF_ADS_TYPE(x) {
        return acos(0.1 - y) + x;
      }));
}

TEST_F(AutoDiffXdTest, Tanh) {
  EXPECT_TRUE(
      Check([](const auto& x, const auto& y) -> DRAKE_NO_REF_ADS_TYPE(x) {
        return tanh(x / x) + y;
      }));
  EXPECT_TRUE(
      Check([](const auto& x, const auto& y) -> DRAKE_NO_REF_ADS_TYPE(x) {
        return tanh(0.1 + x) + y;
      }));
  EXPECT_TRUE(
      Check([](const auto& x, const auto& y) -> DRAKE_NO_REF_ADS_TYPE(x) {
        return tanh(y * y) + x;
      }));
  EXPECT_TRUE(
      Check([](const auto& x, const auto& y) -> DRAKE_NO_REF_ADS_TYPE(x) {
        return tanh(y - 0.1) + x;
      }));
}

TEST_F(AutoDiffXdTest, Sinh) {
  EXPECT_TRUE(
      Check([](const auto& x, const auto& y) -> DRAKE_NO_REF_ADS_TYPE(x) {
        return sinh(x + x) + y;
      }));
  EXPECT_TRUE(
      Check([](const auto& x, const auto& y) -> DRAKE_NO_REF_ADS_TYPE(x) {
        return sinh(x * 0.1) + y;
      }));
  EXPECT_TRUE(
      Check([](const auto& x, const auto& y) -> DRAKE_NO_REF_ADS_TYPE(x) {
        return sinh(y / y) + x;
      }));
  EXPECT_TRUE(
      Check([](const auto& x, const auto& y) -> DRAKE_NO_REF_ADS_TYPE(x) {
        return sinh(0.5 - y) + x;
      }));
}

TEST_F(AutoDiffXdTest, Cosh) {
  EXPECT_TRUE(
      Check([](const auto& x, const auto& y) -> DRAKE_NO_REF_ADS_TYPE(x) {
        return cosh(x * x) + y;
      }));
  EXPECT_TRUE(
      Check([](const auto& x, const auto& y) -> DRAKE_NO_REF_ADS_TYPE(x) {
        return cosh(0.1 / x) + y;
      }));
  EXPECT_TRUE(
      Check([](const auto& x, const auto& y) -> DRAKE_NO_REF_ADS_TYPE(x) {
        return cosh(y + y) + x;
      }));
  EXPECT_TRUE(
      Check([](const auto& x, const auto& y) -> DRAKE_NO_REF_ADS_TYPE(x) {
        return cosh(0.5 - y) + x;
      }));
}

TEST_F(AutoDiffXdTest, min) {
  // Note that x < y and min(x, y) = x in our test harness.
  EXPECT_TRUE(
      Check([](const auto& x, const auto& y) -> DRAKE_NO_REF_ADS_TYPE(x) {
        return min(x + x, y + y) + y;
      }));  // 2x + y
  EXPECT_TRUE(
      Check([](const auto& x, const auto& y) -> DRAKE_NO_REF_ADS_TYPE(x) {
        return min(y + y, x + x) + y;
      }));  // 2x + y
  EXPECT_TRUE(
      Check([](const auto& x, const auto& y) -> DRAKE_NO_REF_ADS_TYPE(x) {
        return min(x + 0.3, y + 0.3) + y;
      }));  // x + 0.3 + y
  EXPECT_TRUE(
      Check([](const auto& x, const auto& y) -> DRAKE_NO_REF_ADS_TYPE(x) {
        return min(y + 0.4, x + 0.4) + y;
      }));  // x + 0.4 + y

  // Note that -y < -x and min(-x, -y) = -y in our test harness.
  EXPECT_TRUE(
      Check([](const auto& x, const auto& y) -> DRAKE_NO_REF_ADS_TYPE(x) {
        return min(-x - x, -y - y) + x;
      }));  // -2y + x
  EXPECT_TRUE(
      Check([](const auto& x, const auto& y) -> DRAKE_NO_REF_ADS_TYPE(x) {
        return min(-y - y, -x - x) + x;
      }));  // -2y + x
  EXPECT_TRUE(
      Check([](const auto& x, const auto& y) -> DRAKE_NO_REF_ADS_TYPE(x) {
        return min(-(x + 0.5), -(y + 0.5)) + x;
      }));  // -y - 0.5 + x
  EXPECT_TRUE(
      Check([](const auto& x, const auto& y) -> DRAKE_NO_REF_ADS_TYPE(x) {
        return min(-(y + 0.6), -(x + 0.6)) + x;
      }));  // -y - 0.6 + x
}

TEST_F(AutoDiffXdTest, max) {
  // Note that x < y and max(x, y) = y in our test harness.
  EXPECT_TRUE(
      Check([](const auto& x, const auto& y) -> DRAKE_NO_REF_ADS_TYPE(x) {
        return max(x + x, y + y) + x;
      }));  // 2y + x
  EXPECT_TRUE(
      Check([](const auto& x, const auto& y) -> DRAKE_NO_REF_ADS_TYPE(x) {
        return max(y + y, x + x) + x;
      }));  // 2y + x
  EXPECT_TRUE(
      Check([](const auto& x, const auto& y) -> DRAKE_NO_REF_ADS_TYPE(x) {
        return max(0.3 + x, 0.3 + y) + x;
      }));  // 0.3 + y + x
  EXPECT_TRUE(
      Check([](const auto& x, const auto& y) -> DRAKE_NO_REF_ADS_TYPE(x) {
        return max(0.4 + y, 0.4 + x) + x;
      }));  // 0.4 + y + x

  // Note that -y < -x and max(-x, -y) = -x in our test harness.
  EXPECT_TRUE(
      Check([](const auto& x, const auto& y) -> DRAKE_NO_REF_ADS_TYPE(x) {
        return max(-x - x, -y - y) + y;
      }));  // -2x + y
  EXPECT_TRUE(
      Check([](const auto& x, const auto& y) -> DRAKE_NO_REF_ADS_TYPE(x) {
        return max(-y - y, -x - x) + y;
      }));  // -2x + y
  EXPECT_TRUE(
      Check([](const auto& x, const auto& y) -> DRAKE_NO_REF_ADS_TYPE(x) {
        return max(0.5 - x, 0.5 - y) + y;
      }));  // 0.5 -x + y
  EXPECT_TRUE(
      Check([](const auto& x, const auto& y) -> DRAKE_NO_REF_ADS_TYPE(x) {
        return max(0.6 - y, 0.6 - x) + y;
      }));  // 0.6 -x + y
}

#undef DRAKE_NO_REF_ADS_TYPE

}  // namespace
}  // namespace common
}  // namespace drake
