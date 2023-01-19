#include <algorithm>
#include <initializer_list>

#include <gmock/gmock.h>

#include "drake/common/ad/auto_diff.h"
#include "drake/common/ad/test/standard_operations_test.h"

namespace drake {
namespace test {
namespace {

constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
constexpr double kInf = std::numeric_limits<double>::infinity();
constexpr double kTol = 1e-14;

// The Eigen reference implementation does not handle any of the IEEE floating-
// point special cases, so we'll use a custom test fixture and directly test our
// AutoDiffDut, rather than try to compare it to a reference implementation.

struct PowCase {
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(PowCase);

  PowCase(double base_in, double exp_in)
      : base(base_in),
        exp(exp_in) {
    expected_value = std::pow(base, exp);

    // Whether any of {base, exp, result} is a NaN.
    is_nan = std::isnan(base) || std::isnan(exp) || std::isnan(expected_value);

    // In some cases, any non-zero element in the grad(base) leads to an
    // ill-defined (i.e., NaN) corresponding element in the grad(result).
    ill_defined_base_grad =
        (base == 0.0) ||
        (!std::isfinite(base)) ||
        (!std::isfinite(exp));

    // In some cases, the grad(base) is unconditionally zeroed out because
    // the result is a constant, and stays the same constant for infinitesimally
    // small changes to the base.
    ignore_base_grad = !ill_defined_base_grad && (exp == 0);

    // In some cases, any non-zero element in the grad(exp) leads to an
    // ill-defined (i.e., NaN) corresponding element in the grad(result).
    ill_defined_exp_grad =
        (base <= 0.0) ||
        (!std::isfinite(base)) ||
        (!std::isfinite(exp));
  }

  double base{};
  double exp{};

  double expected_value{};

  bool is_nan{};
  bool ill_defined_base_grad{};
  bool ignore_base_grad{};
  bool ill_defined_exp_grad{};
};

void PrintTo(const PowCase& pow_case, std::ostream* os) {
  *os << fmt::format("pow({}, {})", pow_case.base, pow_case.exp);
}

std::vector<PowCase> SweepAllCases() {
  std::vector<PowCase> result;

  // A representative sampling of both special and non-special values.
  std::initializer_list<double> sweep = {
    -kInf,
    -2.5,
    -2.0,
    -1.5,
    -1.0,
    -0.5,
    -0.0,
    +0.0,
    0.5,
    1.0,
    1.5,
    2.0,
    2.5,
    kInf,
    kNaN,
  };

  // Let's do all-pairs testing!
  for (double base : sweep) {
    for (double exp : sweep) {
      result.emplace_back(base, exp);
    }
  }

  return result;
}

class PowSpecial : public ::testing::TestWithParam<PowCase> {};

std::string ToAlphaNumeric(double value) {
  if (std::isnan(value)) {
    return "nan";
  }
  if (std::isinf(value)) {
    return std::signbit(value) ? "neg_inf" : "pos_inf";
  }
  if (value == 0) {
    return std::signbit(value) ? "neg_zero" : "pos_zero";
  }
  std::string result = fmt::format("{}", value);
  std::replace(result.begin(), result.end(), '-', 'n');
  std::replace(result.begin(), result.end(), '.', 'd');
  return result;
}

// The test case name must be alphanumeric only (a-z0-9_).
std::string CalcTestName(const testing::TestParamInfo<PowCase>& info) {
  return fmt::format("{}__{}",
      ToAlphaNumeric(info.param.base),
      ToAlphaNumeric(info.param.exp));
}

// Here we sweep the pow(AD,AD) overload only.  We don't test either of the two
// other overloads (AD,double / double,AD) because we know they are implemented
// as one-line wrappers the AD,AD function.
TEST_P(PowSpecial, AdsAds) {
  const PowCase& pow_case = GetParam();
  const AutoDiffDut base(pow_case.base, 3, 1);
  const AutoDiffDut exp(pow_case.exp, 3, 2);

  const AutoDiffDut result = pow(base, exp);
  EXPECT_THAT(result.value(), testing::NanSensitiveDoubleEq(
      pow_case.expected_value));
  ASSERT_EQ(result.derivatives().size(), 3);
  const Eigen::Vector3d grad = result.derivatives();

  // If the result was NaN, then the gradient should be NaN.
  if (pow_case.is_nan) {
    EXPECT_THAT(grad[0], testing::IsNan());
    EXPECT_THAT(grad[1], testing::IsNan());
    EXPECT_THAT(grad[2], testing::IsNan());
    return;
  }

  // Neither base nor exp has an input gradient in the 0th index.
  // That should remain true for the result as well.
  EXPECT_EQ(grad[0], 0.0);

  // The 1st grad index is the partial wrt base.
  if (pow_case.ignore_base_grad) {
    EXPECT_EQ(grad[1], 0.0);
  } else if (pow_case.ill_defined_base_grad) {
    EXPECT_THAT(grad[1], testing::IsNan());
  } else {
    const double expected = pow_case.exp * std::pow(
        pow_case.base, pow_case.exp - 1);
    EXPECT_NEAR(grad[1], expected, kTol);
  }

  // The 2nd grad index is the partial wrt exp.
  if (pow_case.ill_defined_exp_grad) {
    EXPECT_THAT(grad[2], testing::IsNan());
  } else {
    const double expected = pow_case.expected_value * std::log(pow_case.base);
    EXPECT_NEAR(grad[2], expected, kTol);
  }
}

INSTANTIATE_TEST_SUITE_P(StandardOperationsTest, PowSpecial,
                         testing::ValuesIn(SweepAllCases()), &CalcTestName);

}  // namespace
}  // namespace test
}  // namespace drake
