#pragma once

#include <algorithm>
#include <cmath>
#include <limits>

#include <Eigen/Core>
#include <gtest/gtest.h>
#include <unsupported/Eigen/AutoDiff>

#include "drake/common/test_utilities/eigen_matrix_compare.h"

namespace drake {
namespace test {

// While we are transitioning between the old and new auto diff implementations
// (`Eigen::AutoDiffScalar<Eigen::VectorXd>` and `drake::ad::AutoDiff`) this
// DRAKE_AUTODIFFXD_DUT preprocessor macro allows us to reuse the
// StandardOperationsTest fixture and its associated macros for both
// implementations' test suites.
//
// The value used for DRAKE_AUTODIFFXD_DUT is defined by the build system as
// either `drake::AutoDiffXd` or `drake::ad::AutoDiff` for the old and new
// implementations, respectively (via `copts` in `common/BUILD.bazel` and
// `common/ad/BUILD.bazel`).
//
// Once we finally drop the old implementation, we should undo the use of
// the preprocessor for this alias.
using AutoDiffDut = DRAKE_AUTODIFFXD_DUT;

// This is Eigen's "reference implementation" of autodiff. We'll compare our
// AutoDiff results to Eigen's AutoDiffScalar results for unit testing.
using AutoDiff3 = Eigen::AutoDiffScalar<Eigen::Matrix<double, 3, 1>>;

class StandardOperationsTest : public ::testing::Test {
 protected:
  ::testing::AssertionResult CheckAdsEqual(
      const AutoDiffDut& actual,
      const AutoDiff3& expected) {
    // Pack the results into a 4-vector (for easier comparison and reporting).
    Eigen::Vector4d actual4;
    actual4.setZero();
    actual4(0) = actual.value();
    // When an AutoDiff derivatives vector is empty, the implication is that
    // that all derivatives are zero.
    if (actual.derivatives().size() > 0) {
       actual4.tail(3) = actual.derivatives();
    }

    Eigen::Vector4d expected4;
    expected4.setZero();
    expected4(0) = expected.value();
    expected4.tail(3) = expected.derivatives();

    return CompareMatrices(
        actual4, expected4,
        10 * std::numeric_limits<double>::epsilon(),
        MatrixCompareType::relative)
      << "\n(where actual.derivatives()size() = "
      << actual.derivatives().size() << ")";
  }

  // Evaluates a given function f with values of AutoDiffXd and values with
  // AutoDiffd<3>. It checks if the values and the derivatives of those
  // evaluation results are matched.
  template <typename F>
  ::testing::AssertionResult Check(const F& f) {
    // AutoDiffXd constants -- x and y.
    const AutoDiffDut x_xd{0.4};
    AutoDiffDut y_xd{0.3};

    // AutoDiffd<3> constants -- x and y.
    const AutoDiff3 x_3d{x_xd.value()};
    AutoDiff3 y_3d{y_xd.value()};

    // We only set the derivatives of y and leave x's uninitialized.
    y_xd.derivatives() = Eigen::VectorXd::Ones(3);
    y_3d.derivatives() = Eigen::Vector3d::Ones();

    // Compute the autodiff results.
    AutoDiffDut e_xd{f(x_xd, y_xd)};
    AutoDiff3 e_3d{f(x_3d, y_3d)};

    // When either value is not finite, the derivatives are allowed to be
    // nonsense.
    if (!std::isfinite(e_xd.value()) || !std::isfinite(e_3d.value())) {
      e_xd.derivatives() = Eigen::Vector3d::Constant(NAN);
      e_3d.derivatives() = Eigen::Vector3d::Constant(NAN);
    }

    // Compare the results.
    return CheckAdsEqual(e_xd, e_3d);
  }
};

// We need to specify the return type of the polymorphic lambda function that is
// passed to StandardOperationsTest::Check() method.
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
// etc. whose arguments must be in [-1, 1].
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

#define CHECK_BINARY_FUNCTION_ADS_SCALAR(f, x, y, c) \
  CHECK_EXPR(f(x, c) + y);                           \
  CHECK_EXPR(f(x + x, c) + y);                       \
  CHECK_EXPR(f(x + y, c) + y);                       \
  CHECK_EXPR(f(x - x + 5.0, c) - y);                 \
  CHECK_EXPR(f(x * x, c) * y);                       \
  CHECK_EXPR(f(x / x, c) / y);                       \
  CHECK_EXPR(f(x + c, c) + y);                       \
  CHECK_EXPR(f(c + x, c) + y);                       \
  CHECK_EXPR(f(x * c, c) + y);                       \
  CHECK_EXPR(f(c * x, c) + y);                       \
  CHECK_EXPR(f(-x, c) + y);

#define CHECK_BINARY_FUNCTION_SCALAR_ADS(f, x, y, c) \
  CHECK_EXPR(f(c, x) + y);                           \
  CHECK_EXPR(f(c, x + x) + y);                       \
  CHECK_EXPR(f(c, x + y) + y);                       \
  CHECK_EXPR(f(c, x - x + 5.0) - y);                 \
  CHECK_EXPR(f(c, x * x) * y);                       \
  CHECK_EXPR(f(c, x / x) / y);                       \
  CHECK_EXPR(f(c, x + c) + y);                       \
  CHECK_EXPR(f(c, c + x) + y);                       \
  CHECK_EXPR(f(c, x * c) + y);                       \
  CHECK_EXPR(f(c, c * x) + y);                       \
  CHECK_EXPR(f(c, -x) + y);

}  // namespace test
}  // namespace drake
