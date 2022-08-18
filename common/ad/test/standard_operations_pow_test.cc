#include "drake/common/ad/auto_diff.h"
#include "drake/common/ad/test/standard_operations_test.h"

namespace drake {
namespace test {
namespace {

// Note that standard_operations_pow_special_test has separate tests for all of
// the special cases (infinities, zeros, NaNs, etc.). This test only covers the
// everyday cases with real numbers where the chain rule happens as expected.

// For the pow(ADS,double), Eigen provides a reference implementation that we
// can compare against.
TEST_F(StandardOperationsTest, PowAdsDouble) {
  CHECK_BINARY_FUNCTION_ADS_SCALAR(pow, x, y, 0.3);
  CHECK_BINARY_FUNCTION_ADS_SCALAR(pow, x, y, -0.3);
  CHECK_BINARY_FUNCTION_ADS_SCALAR(pow, y, x, 0.4);
  CHECK_BINARY_FUNCTION_ADS_SCALAR(pow, y, x, -0.4);
}

// Eigen does not provide an implementation of pow(ADS,ADS) nor pow(double,ADS)
// for us to compare against, so we'll need to get creative.
//
// For starters, when the exponent has an empty gradient the results should be
// equivalent to pow(ADS,double).
AutoDiffDut pow_no_exp_grad(const AutoDiffDut& base, const AutoDiffDut& exp) {
  return pow(base, exp);
}
AutoDiffDut pow_no_exp_grad(double base, const AutoDiffDut& exp) {
  return pow(base, exp);
}
AutoDiff3 pow_no_exp_grad(const AutoDiff3& base, const AutoDiff3& exp) {
  DRAKE_DEMAND(exp.derivatives().isZero(0.0));
  return pow(base, exp.value());
}
AutoDiff3 pow_no_exp_grad(double base, const AutoDiff3& exp) {
  DRAKE_DEMAND(exp.derivatives().isZero(0.0));
  return std::pow(base, exp.value());
}

TEST_F(StandardOperationsTest, PowNoExpGradAdsAds) {
  // N.B. In our text fixture, `x` has an empty partial derivatives vector.
  // We'll use only expressions over x (and not y) as an exponent.
  CHECK_EXPR(pow_no_exp_grad(y, x));
  CHECK_EXPR(pow_no_exp_grad(y, x + 1.0));
  CHECK_EXPR(pow_no_exp_grad(y, x - 1.0));
  CHECK_EXPR(pow_no_exp_grad(x + y, x));
  CHECK_EXPR(pow_no_exp_grad(x + y, x + 1.0));
  CHECK_EXPR(pow_no_exp_grad(x + y, x - 1.0));
  CHECK_EXPR(pow_no_exp_grad(5.0 * (x + y), 3.0 * x));
}

TEST_F(StandardOperationsTest, PowNoExpGradDoubleAds) {
  // N.B. In our text fixture, `x` has an empty partial derivatives vector.
  // We'll use only expressions over x (and not y) as an exponent.
  CHECK_EXPR(pow_no_exp_grad(0.3, x));
  CHECK_EXPR(pow_no_exp_grad(0.3, x + 1.0));
  CHECK_EXPR(pow_no_exp_grad(0.3, x - 1.0));
  CHECK_EXPR(pow_no_exp_grad(0.3, 3.0 * x));
}

}  // namespace
}  // namespace test
}  // namespace drake
