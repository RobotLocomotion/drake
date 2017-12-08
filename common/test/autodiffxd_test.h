#pragma once

#include <gtest/gtest.h>

#include "drake/common/autodiff.h"

namespace drake {
namespace test {

class AutoDiffXdTest : public ::testing::Test {
 protected:
  void SetUp() override {
    // We only set the derivatives of y and leave x's uninitialized.
    y_xd_.derivatives() = Eigen::VectorXd::Ones(3);
    y_3d_.derivatives() = Eigen::Vector3d::Ones();
  }

  // Evaluates a given function f with values of AutoDiffXd and values with
  // AutoDiffd<3>. It checks if the values and the derivatives of those
  // evaluation results are matched.
  template <typename F>
  ::testing::AssertionResult Check(const F& f) {
    const AutoDiffXd e1{f(x_xd_, y_xd_)};
    const AutoDiffd<3> e2{f(x_3d_, y_3d_)};

    if (std::isnan(e1.value()) && std::isnan(e2.value())) {
      // Both values are NaN.
      return ::testing::AssertionSuccess();
    }
    if (e1.value() != e2.value()) {
      return ::testing::AssertionFailure()
             << "Values do not match: " << e1.value() << " and " << e2.value();
    }
    if (e1.derivatives().array().isNaN().all() &&
        e2.derivatives().array().isNaN().all()) {
      // Both derivatives are NaN.
      return ::testing::AssertionSuccess();
    }
    if (e1.derivatives() != e2.derivatives()) {
      return ::testing::AssertionFailure() << "Derivatives do not match:\n"
                                           << e1.derivatives() << "\n----\n"
                                           << e2.derivatives() << "\n";
    }
    return ::testing::AssertionSuccess();
  }

  // AutoDiffXd constants -- x and y.
  const AutoDiffXd x_xd_{0.4};
  AutoDiffXd y_xd_{0.3};
  // AutoDiffd<3> constants -- x and y.
  const AutoDiffd<3> x_3d_{x_xd_.value()};
  AutoDiffd<3> y_3d_{y_xd_.value()};
};

// We need to specify the return type of the polymorphic lambda function that is
// passed to AutoDiffXdTest::Check() method.
#define CHECK_EXPR(expr)                                                    \
  EXPECT_TRUE(                                                              \
      Check([](const auto& x, const auto& y) ->                             \
            typename Eigen::internal::remove_reference<decltype(x)>::type { \
              return expr;                                                  \
            }))                                                             \
      << #expr  // Print statement to locate it if it fails

#define CHECK_BINARY_OP(bop, x, y, c) \
  CHECK_EXPR((x bop x)bop(y bop y));  \
  CHECK_EXPR((x bop y)bop(x bop y));  \
  CHECK_EXPR((x bop y)bop c);         \
  CHECK_EXPR((x bop y)bop c);         \
  CHECK_EXPR((x bop c)bop y);         \
  CHECK_EXPR((c bop x)bop y);         \
  CHECK_EXPR(x bop(y bop c));         \
  CHECK_EXPR(x bop(c bop y));         \
  CHECK_EXPR(c bop(x bop y));

#define CHECK_UNARY_FUNCTION(f, x, y, c) \
  CHECK_EXPR(f(x + x) + (y + y));        \
  CHECK_EXPR(f(x + y) + (x + y));        \
  CHECK_EXPR(f(x - x) + (y - y));        \
  CHECK_EXPR(f(x - y) + (x - y));        \
  CHECK_EXPR(f(x* x) + (y * y));         \
  CHECK_EXPR(f(x* y) + (x * y));         \
  CHECK_EXPR(f(x / x) + (y / y));        \
  CHECK_EXPR(f(x / y) + (x / y));        \
  CHECK_EXPR(f(x + c) + y);              \
  CHECK_EXPR(f(x - c) + y);              \
  CHECK_EXPR(f(x* c) + y);               \
  CHECK_EXPR(f(x / c) + y);              \
  CHECK_EXPR(f(c + x) + y);              \
  CHECK_EXPR(f(c - x) + y);              \
  CHECK_EXPR(f(c* x) + y);               \
  CHECK_EXPR(f(c / x) + y);              \
  CHECK_EXPR(f(-x) + y);

#define CHECK_BINARY_FUNCTION_ADS_ADS(f, x, y, c) \
  CHECK_EXPR(f(x + x, y + y) + x);                \
  CHECK_EXPR(f(x + x, y + y) + y);                \
  CHECK_EXPR(f(x + y, y + y) + x);                \
  CHECK_EXPR(f(x + y, y + y) + y);                \
  CHECK_EXPR(f(x - x, y - y) - x);                \
  CHECK_EXPR(f(x - x, y - y) - y);                \
  CHECK_EXPR(f(x - y, y - y) - x);                \
  CHECK_EXPR(f(x - y, y - y) - y);                \
  CHECK_EXPR(f(x* x, y* y) * x);                  \
  CHECK_EXPR(f(x* x, y* y) * y);                  \
  CHECK_EXPR(f(x* y, y* y) * x);                  \
  CHECK_EXPR(f(x* y, y* y) * y);                  \
  CHECK_EXPR(f(x / x, y / y) / x);                \
  CHECK_EXPR(f(x / x, y / y) / y);                \
  CHECK_EXPR(f(x / y, y / y) / x);                \
  CHECK_EXPR(f(x / y, y / y) / y);                \
  CHECK_EXPR(f(x + c, y + c) + x);                \
  CHECK_EXPR(f(c + x, c + x) + x);                \
  CHECK_EXPR(f(x* c, y* c) + x);                  \
  CHECK_EXPR(f(c* x, c* x) + x);                  \
  CHECK_EXPR(f(-x, -y) + y)

#define CHECK_BINARY_FUNCTION_ADS_SCALAR(f, x, y, c) \
  CHECK_EXPR(f(x, c) + y);                           \
  CHECK_EXPR(f(x + x, c) + y);                       \
  CHECK_EXPR(f(x + y, c) + y);                       \
  CHECK_EXPR(f(x - x, c) - y);                       \
  CHECK_EXPR(f(x - x, c) - y);                       \
  CHECK_EXPR(f(x* x, c) * y);                        \
  CHECK_EXPR(f(x* x, c) * y);                        \
  CHECK_EXPR(f(x / x, c) / y);                       \
  CHECK_EXPR(f(x / x, c) / y);                       \
  CHECK_EXPR(f(x + c, c) + y);                       \
  CHECK_EXPR(f(c + x, c) + y);                       \
  CHECK_EXPR(f(x* c, c) + y);                        \
  CHECK_EXPR(f(c* x, c) + y);                        \
  CHECK_EXPR(f(-x, c) + y);

}  // namespace test
}  // namespace drake
