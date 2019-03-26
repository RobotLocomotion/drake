#pragma once

#include <algorithm>
#include <limits>

#include <gtest/gtest.h>

#include "drake/common/autodiff.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"

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

    // Compute the expression results.
    const AutoDiffXd e_xd{f(x_xd, y_xd)};
    const AutoDiffd<3> e_3d{f(x_3d, y_3d)};

    // Pack the results into a 4-vector (for easier comparison and reporting).
    Eigen::Vector4d value_and_der_x;
    Eigen::Vector4d value_and_der_3;
    value_and_der_x.setZero();
    value_and_der_3.setZero();
    value_and_der_x(0) = e_xd.value();
    value_and_der_3(0) = e_3d.value();

    // When the values are finite, compare the derivatives.  When the value are
    // not finite, then derivatives are allowed to be nonsense.
    if (std::isfinite(e_xd.value()) && std::isfinite(e_3d.value())) {
      value_and_der_3.tail(3) = e_3d.derivatives();
      // When an AutoDiffXd derivatives vector is empty, the implication is
      // that all derivatives are zero.
      if (e_xd.derivatives().size() > 0) {
        value_and_der_x.tail(3) = e_xd.derivatives();
      }
    } else {
      value_and_der_3.tail(3) = Eigen::Vector3d::Constant(NAN);
      value_and_der_x.tail(3) = Eigen::Vector3d::Constant(NAN);
    }

    return CompareMatrices(
        value_and_der_x, value_and_der_3,
        2.0 * std::numeric_limits<double>::epsilon(),
        MatrixCompareType::relative)
      << "\n(where xd.size() = " << e_xd.derivatives().size() << ")";
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
