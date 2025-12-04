#include <algorithm>
#include <initializer_list>
#include <limits>
#include <string>
#include <vector>

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

  PowCase(double base_in, double exp_in) : base(base_in), exp(exp_in) {
    expected_value = std::pow(base, exp);

    // From the header doc:
    //
    // The resulting partial derivative ∂/∂vᵢ is undefined (i.e., NaN) for all
    // of the following cases:
    //
    // - ∂base/∂vᵢ is non-zero and either:
    //   - base, exp, or pow(base, exp) not finite, or
    //   - base is 0 and exp < 0
    const bool non_finite = !std::isfinite(base) || !std::isfinite(exp) ||
                            !std::isfinite(expected_value);
    ill_defined_base_grad = non_finite || ((base == 0) && (exp < 0));

    // - ∂exp/∂vᵢ is non-zero and either:
    //   - base, exp, or pow(base, exp) not finite, or
    //   - base < 0.
    ill_defined_exp_grad = non_finite || (base < 0);
  }

  double base{};
  double exp{};

  double expected_value{};

  bool ill_defined_base_grad{};
  bool ill_defined_exp_grad{};
};

void PrintTo(const PowCase& pow_case, std::ostream* os) {
  *os << fmt::format("pow({}, {})", pow_case.base, pow_case.exp);
}

std::vector<PowCase> SweepAllCases() {
  std::vector<PowCase> result;

  // A representative sampling of both special and non-special values.
  std::initializer_list<double> sweep = {
      -kInf, -3.0, -2.5, -2.0, -1.5, -1.0, -0.5, -0.0, +0.0,
      0.5,   1.0,  1.5,  2.0,  2.5,  3.0,  kInf, kNaN,
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
  return fmt::format("{}__{}", ToAlphaNumeric(info.param.base),
                     ToAlphaNumeric(info.param.exp));
}

// Here we sweep the pow(AD,AD) overload only.  We don't test either of the two
// other overloads (AD,double / double,AD) because we know they are implemented
// as one-line wrappers the AD,AD function.
TEST_P(PowSpecial, AdsAds) {
  const PowCase& pow_case = GetParam();
  const double b = pow_case.base;
  const double x = pow_case.exp;
  const AutoDiffDut base(b, 3, 1);
  const AutoDiffDut exp(x, 3, 2);

  const AutoDiffDut result = pow(base, exp);
  EXPECT_THAT(result.value(),
              testing::NanSensitiveDoubleEq(pow_case.expected_value));
  ASSERT_EQ(result.derivatives().size(), 3);
  const Eigen::Vector3d grad = result.derivatives();

  // Neither base nor exp has an input gradient in the 0th index.
  // That should remain true for the result as well.
  EXPECT_EQ(grad[0], 0.0);

  // The 1st grad index is the partial wrt base.
  if (pow_case.ill_defined_base_grad) {
    EXPECT_THAT(grad[1], testing::IsNan());
  } else {
    const double expected = (x == 0) ? 0.0 : x * std::pow(b, x - 1);
    EXPECT_NEAR(grad[1], expected, kTol);
  }

  // The 2nd grad index is the partial wrt exp.
  if (pow_case.ill_defined_exp_grad) {
    EXPECT_THAT(grad[2], testing::IsNan());
  } else {
    const double expected = (b == 0 && x == 0) ? -kInf
                            : (b == 0 && x > 0)
                                ? 0.0
                                : pow_case.expected_value * std::log(b);
    EXPECT_NEAR(grad[2], expected, kTol);
  }
}

INSTANTIATE_TEST_SUITE_P(StandardOperationsTest, PowSpecial,
                         testing::ValuesIn(SweepAllCases()), &CalcTestName);

}  // namespace
}  // namespace test
}  // namespace drake
