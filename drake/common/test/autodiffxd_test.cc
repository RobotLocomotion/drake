#include "drake/common/autodiffxd.h"

#include <gtest/gtest.h>

#include "drake/common/eigen_autodiff_types.h"

namespace drake {
namespace common {
namespace {

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

TEST_F(AutoDiffXdTest, Addition) {
  CHECK_BINARY_OP(+, x, y, 1.0);
  CHECK_BINARY_OP(+, x, y, -1.0);
  CHECK_BINARY_OP(+, y, x, 1.0);
  CHECK_BINARY_OP(+, y, x, -1.0);
}

TEST_F(AutoDiffXdTest, Subtraction) {
  CHECK_BINARY_OP(-, x, y, 1.0);
  CHECK_BINARY_OP(-, x, y, -1.0);
  CHECK_BINARY_OP(-, y, x, 1.0);
  CHECK_BINARY_OP(-, y, x, -1.0);
}

TEST_F(AutoDiffXdTest, Multiplication) {
  CHECK_BINARY_OP(*, x, y, 1.0);
  CHECK_BINARY_OP(*, x, y, -1.0);
  CHECK_BINARY_OP(*, y, x, 1.0);
  CHECK_BINARY_OP(*, y, x, -1.0);
}

TEST_F(AutoDiffXdTest, Division) {
  CHECK_BINARY_OP(/, x, y, 1.0);
  CHECK_BINARY_OP(/, x, y, -1.0);
  CHECK_BINARY_OP(/, y, x, 1.0);
  CHECK_BINARY_OP(/, y, x, -1.0);
}
#undef CHECK_BINARY_OP

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

TEST_F(AutoDiffXdTest, Abs) {
  CHECK_UNARY_FUNCTION(abs, x, y, 0.1);
  CHECK_UNARY_FUNCTION(abs, x, y, -0.1);
  CHECK_UNARY_FUNCTION(abs, y, x, 0.1);
  CHECK_UNARY_FUNCTION(abs, y, x, -0.1);
}

TEST_F(AutoDiffXdTest, Abs2) {
  CHECK_UNARY_FUNCTION(abs2, x, y, 0.1);
  CHECK_UNARY_FUNCTION(abs2, x, y, -0.1);
  CHECK_UNARY_FUNCTION(abs2, y, x, 0.1);
  CHECK_UNARY_FUNCTION(abs2, y, x, -0.1);
}

TEST_F(AutoDiffXdTest, Sqrt) {
  CHECK_UNARY_FUNCTION(sqrt, x, y, 0.1);
  CHECK_UNARY_FUNCTION(sqrt, x, y, -0.1);
  CHECK_UNARY_FUNCTION(sqrt, y, x, 0.1);
  CHECK_UNARY_FUNCTION(sqrt, y, x, -0.1);
}

TEST_F(AutoDiffXdTest, Cos) {
  CHECK_UNARY_FUNCTION(cos, x, y, 0.1);
  CHECK_UNARY_FUNCTION(cos, x, y, -0.1);
  CHECK_UNARY_FUNCTION(cos, y, x, 0.1);
  CHECK_UNARY_FUNCTION(cos, y, x, -0.1);
}

TEST_F(AutoDiffXdTest, Sin) {
  CHECK_UNARY_FUNCTION(sin, x, y, 0.1);
  CHECK_UNARY_FUNCTION(sin, x, y, -0.1);
  CHECK_UNARY_FUNCTION(sin, y, x, 0.1);
  CHECK_UNARY_FUNCTION(sin, y, x, -0.1);
}

TEST_F(AutoDiffXdTest, Exp) {
  CHECK_UNARY_FUNCTION(exp, x, y, 0.1);
  CHECK_UNARY_FUNCTION(exp, x, y, -0.1);
  CHECK_UNARY_FUNCTION(exp, y, x, 0.1);
  CHECK_UNARY_FUNCTION(exp, y, x, -0.1);
}

TEST_F(AutoDiffXdTest, Log) {
  CHECK_UNARY_FUNCTION(log, x, y, 0.1);
  CHECK_UNARY_FUNCTION(log, x, y, -0.1);
  CHECK_UNARY_FUNCTION(log, y, x, 0.1);
  CHECK_UNARY_FUNCTION(log, y, x, -0.1);
}

TEST_F(AutoDiffXdTest, Tan) {
  CHECK_UNARY_FUNCTION(tan, x, y, 0.1);
  CHECK_UNARY_FUNCTION(tan, x, y, -0.1);
  CHECK_UNARY_FUNCTION(tan, y, x, 0.1);
  CHECK_UNARY_FUNCTION(tan, y, x, -0.1);
}

TEST_F(AutoDiffXdTest, Asin) {
  CHECK_UNARY_FUNCTION(asin, x, y, 0.1);
  CHECK_UNARY_FUNCTION(asin, x, y, -0.1);
  CHECK_UNARY_FUNCTION(asin, y, x, 0.1);
  CHECK_UNARY_FUNCTION(asin, y, x, -0.1);
}

TEST_F(AutoDiffXdTest, Acos) {
  CHECK_UNARY_FUNCTION(acos, x, y, 0.1);
  CHECK_UNARY_FUNCTION(acos, x, y, -0.1);
  CHECK_UNARY_FUNCTION(acos, y, x, 0.1);
  CHECK_UNARY_FUNCTION(acos, y, x, -0.1);
}

TEST_F(AutoDiffXdTest, Tanh) {
  CHECK_UNARY_FUNCTION(tanh, x, y, 0.1);
  CHECK_UNARY_FUNCTION(tanh, x, y, -0.1);
  CHECK_UNARY_FUNCTION(tanh, y, x, 0.1);
  CHECK_UNARY_FUNCTION(tanh, y, x, -0.1);
}

TEST_F(AutoDiffXdTest, Sinh) {
  CHECK_UNARY_FUNCTION(sinh, x, y, 0.1);
  CHECK_UNARY_FUNCTION(sinh, x, y, -0.1);
  CHECK_UNARY_FUNCTION(sinh, y, x, 0.1);
  CHECK_UNARY_FUNCTION(sinh, y, x, -0.1);
}

TEST_F(AutoDiffXdTest, Cosh) {
  CHECK_UNARY_FUNCTION(cosh, x, y, 0.1);
  CHECK_UNARY_FUNCTION(cosh, x, y, -0.1);
  CHECK_UNARY_FUNCTION(cosh, y, x, 0.1);
  CHECK_UNARY_FUNCTION(cosh, y, x, -0.1);
}
#undef CHECK_UNARY_FUNCTION

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

TEST_F(AutoDiffXdTest, Atan2) {
  CHECK_BINARY_FUNCTION_ADS_ADS(atan2, x, y, 0.3);
  CHECK_BINARY_FUNCTION_ADS_ADS(atan2, x, y, -0.3);
  CHECK_BINARY_FUNCTION_ADS_ADS(atan2, y, x, 0.4);
  CHECK_BINARY_FUNCTION_ADS_ADS(atan2, y, x, -0.4);
}

TEST_F(AutoDiffXdTest, Min) {
  CHECK_BINARY_FUNCTION_ADS_ADS(min, x, y, 0.3);
  CHECK_BINARY_FUNCTION_ADS_ADS(min, x, y, -0.3);
  CHECK_BINARY_FUNCTION_ADS_ADS(min, y, x, 0.4);
  CHECK_BINARY_FUNCTION_ADS_ADS(min, y, x, -0.4);
}

TEST_F(AutoDiffXdTest, Max) {
  CHECK_BINARY_FUNCTION_ADS_ADS(max, x, y, 0.5);
  CHECK_BINARY_FUNCTION_ADS_ADS(max, x, y, -0.3);
  CHECK_BINARY_FUNCTION_ADS_ADS(max, y, x, 0.4);
  CHECK_BINARY_FUNCTION_ADS_ADS(max, y, x, -0.4);
}

TEST_F(AutoDiffXdTest, Pow1) {
  CHECK_BINARY_FUNCTION_ADS_ADS(pow, x, y, 0.3);
  CHECK_BINARY_FUNCTION_ADS_ADS(pow, x, y, -0.3);
  CHECK_BINARY_FUNCTION_ADS_ADS(pow, y, x, 0.4);
  CHECK_BINARY_FUNCTION_ADS_ADS(pow, y, x, -0.4);
}

#undef CHECK_BINARY_FUNCTION_ADS_ADS

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

TEST_F(AutoDiffXdTest, Pow2) {
  CHECK_BINARY_FUNCTION_ADS_SCALAR(pow, x, y, 0.3);
  CHECK_BINARY_FUNCTION_ADS_SCALAR(pow, x, y, -0.3);
  CHECK_BINARY_FUNCTION_ADS_SCALAR(pow, y, x, 0.4);
  CHECK_BINARY_FUNCTION_ADS_SCALAR(pow, y, x, -0.4);
  // Note that Eigen's AutoDiffScalar does not provide an implementation for
  // pow(double, ADS). Therefore, we do not provide tests for that here.
}

#undef CHECK_BINARY_FUNCTION_ADS_SCALAR
#undef CHECK_EXPR

}  // namespace
}  // namespace common
}  // namespace drake
