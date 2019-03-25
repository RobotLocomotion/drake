#pragma once

#include <algorithm>
#include <limits>

#include <gtest/gtest.h>

#include "drake/common/autodiff.h"

namespace drake {
namespace test {

class AutoDiffXdTest : public ::testing::Test {
 protected:
  // Evaluates a given function f with values of AutoDiffXd and values with
  // AutoDiffd<3>. It checks if the values and the derivatives of those
  // evaluation results are matched.
  template <typename F>
  ::testing::AssertionResult Check(const F& f) {
    // AutoDiffXd constants -- x and y.
    const AutoDiffXd x_xd{0.4};
    AutoDiffXd y_xd{0.3};
    // AutoDiffd<3> constants -- x and y.
    const AutoDiffd<3> x_3d{x_xd.value()};
    AutoDiffd<3> y_3d{y_xd.value()};
    // We only set the derivatives of y and leave x's uninitialized.
    y_xd.derivatives() = Eigen::VectorXd::Ones(3);
    y_3d.derivatives() = Eigen::Vector3d::Ones();

    const AutoDiffXd e_xd{f(x_xd, y_xd)};
    const AutoDiffd<3> e_3d{f(x_3d, y_3d)};

    if (std::isnan(e_xd.value()) && std::isnan(e_3d.value())) {
      // Both values are NaN.
      return ::testing::AssertionSuccess();
    }
    // Slightly different execution branches might lead to results that differ
    // by a machine precision order of magnitude.
    const double kEpsilon = std::numeric_limits<double>::epsilon();
    const double kValueTolerance =
        10 * kEpsilon *
        std::max(std::abs(e_xd.value()), std::abs(e_3d.value()));
    if (std::abs(e_xd.value() - e_3d.value()) > kValueTolerance) {
      return ::testing::AssertionFailure()
             << "Values do not match: " << e_xd.value() << " and "
             << e_3d.value();
    }
    if (e_xd.derivatives().array().isNaN().all() &&
        e_3d.derivatives().array().isNaN().all()) {
      // Both derivatives are NaN.
      return ::testing::AssertionSuccess();
    }
    const double kDerivativeTolerance = 10 * kEpsilon;  // relative tolerance.
    const bool derivatives_match =
        // Zero derivatives.
        (e_xd.derivatives().size() == 0 &&
         e_3d.derivatives().isZero(kDerivativeTolerance)) ||
        // Non-zero derivatives
        e_xd.derivatives().isApprox(e_3d.derivatives(), kDerivativeTolerance);
    if (!derivatives_match) {
      return ::testing::AssertionFailure() << "Derivatives do not match:\n"
                                           << e_xd.derivatives() << "\n----\n"
                                           << e_3d.derivatives() << "\n";
    }
    return ::testing::AssertionSuccess();
  }
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

// The multiplicative factor 0.9 < 1.0 let us call function such as asin, acos,
// etc. whose arguments must be in (-1, 1).
// The additive factor 5.0 let us call functions whose arguments must be
// positive.
#define CHECK_UNARY_FUNCTION(f, x, y, c) \
  CHECK_EXPR(f(x + x) + (y + y));        \
  CHECK_EXPR(f(x + y) + (x + y));        \
  CHECK_EXPR(f(x - x + 5.0) + (y - y));  \
  CHECK_EXPR(f(x - y + 5.0) + (x - y));  \
  CHECK_EXPR(f(x * x) + (y * y));        \
  CHECK_EXPR(f(x * y) + (x * y));        \
  CHECK_EXPR(f(0.9 * x / x) + (y / y));  \
  CHECK_EXPR(f(x / y) + (x / y));        \
  CHECK_EXPR(f(x + c) + y);              \
  CHECK_EXPR(f(x - c + 5.0) + y);        \
  CHECK_EXPR(f(x * c + 5.0) + y);        \
  CHECK_EXPR(f(x + 5.0) + y / c);        \
  CHECK_EXPR(f(c + x + 5.0) + y);        \
  CHECK_EXPR(f(c - x + 5.0) + y);        \
  CHECK_EXPR(f(c * x + 5.0) + y);        \
  CHECK_EXPR(f(c / x  + 5.0) + y);       \
  CHECK_EXPR(f(-x  + 5.0) + y);

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
  CHECK_EXPR(f(c + x, c + x) + y);                \
  CHECK_EXPR(f(x* c, y* c) + x);                  \
  CHECK_EXPR(f(c* x, c* x) + y);                  \
  CHECK_EXPR(f(-x, -y) + y)

// The additive factor 5.0 let us call functions whose arguments must be
// positive in order to have well defined derivatives. Eg.: sqrt, pow.
#define CHECK_BINARY_FUNCTION_ADS_SCALAR(f, x, y, c) \
  CHECK_EXPR(f(x, c) + y);                           \
  CHECK_EXPR(f(x + x, c) + y);                       \
  CHECK_EXPR(f(x + y, c) + y);                       \
  CHECK_EXPR(f(x - x + 5.0, c) - y);                 \
  CHECK_EXPR(f(x - x + 5.0, c) - y);                 \
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
