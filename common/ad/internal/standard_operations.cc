// Our header file doesn't stand on its own; it should only ever be included
// indirectly via auto_diff.h

/* clang-format off to disable clang-format-includes */
// NOLINTNEXTLINE(build/include)
#include "drake/common/ad/auto_diff.h"
/* clang-format on */

#include <ostream>

#include <fmt/format.h>

namespace drake {
namespace ad {

namespace {

constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

bool HasGrad(const AutoDiff& x) {
  return !x.partials().make_const_xpr().isZero(0.0);
}

bool IsPosInf(const AutoDiff& x) {
  return std::isinf(x.value()) && (x.value() > 0);
}

bool IsNegInf(const AutoDiff& x) {
  return std::isinf(x.value()) && (x.value() < 0);
}

bool IsInteger(const AutoDiff& x) {
  return std::isfinite(x.value())
      && (std::trunc(x.value()) == x.value());
}

bool IsOddInteger(const AutoDiff& x) {
  return std::isfinite(x.value())
      && (std::fmod(std::abs(x.value()), 2.0) == 1.0);
}

}  // namespace

AutoDiff pow(AutoDiff a, const AutoDiff& b) {
  // Compute the new a.value() up front; we'll use it throughout.
  const double new_value = std::pow(a.value(), b.value());

  // Refer to https://en.cppreference.com/w/cpp/numeric/math/pow for the 22
  // special cases we need to handle here before we move on to the chain rule.

  // Rule #9: pow(+1, exp) returns 1 for any exp, even when exp is NaN.
  if (a.value() == 1) {
    // The resulting gradient is only well-defined when `a` had no gradient.
    const double grad = HasGrad(a) ? kNaN : 0.0;
    a.partials().SetConstantMatchingSize(grad, b.partials());
    a.value() = new_value;
    return a;
  }

  // Rule #10: pow(base, ±0) returns 1 for any base, even when base is NaN.
  if (b.value() == 0) {
    // The resulting gradient is only well-defined when `b` had no gradient.
    const double grad = HasGrad(b) ? kNaN : 0.0;
    a.partials().SetConstantMatchingSize(grad, b.partials());
    a.value() = new_value;
    return a;
  }

  // Rule #22: except [for the] above, if any argument is NaN, NaN is returned.
  if (std::isnan(a.value()) || std::isnan(b.value())) {
    const double grad = kNaN;
    a.partials().SetConstantMatchingSize(grad, b.partials());
    a.value() = new_value;
    return a;
  }

  // --- As of this point, both arguments are known to be non-NaN. ---
  DRAKE_ASSERT(!std::isnan(a.value()));
  DRAKE_ASSERT(!std::isnan(b.value()));

  // Rule #20: pow(+∞, exp) returns +0 for any negative exp.
  // Rule #21: pow(+∞, exp) returns +∞ for any positive exp.
  // Note that Rule #10 above already covered when exp == 0.
  // Note that Rule #22 above already covered when exp is NaN.
  if (IsPosInf(a)) {
    // The resulting gradient is well-defined everywhere.
    const double grad = 0.0;
    a.partials().SetConstantMatchingSize(grad, b.partials());
    a.value() = new_value;
    return a;
  }

  // Rule #16: pow(-∞, exp) returns -0 if exp is a negative odd integer.
  // Rule #18: pow(-∞, exp) returns -∞ if exp is a positive odd integer.
  // Rule #17: pow(-∞, exp) returns +0 if exp is a negative non-integer or
  //   negative even integer.
  // Rule #19: pow(-∞, exp) returns +∞ if exp is a positive non-integer or
  //   positive even integer.
  if (IsNegInf(a)) {
    // The resulting gradient is ill-defined for #16 and #18.
    const double grad = IsOddInteger(b) ? kNaN : 0.0;
    a.partials().SetConstantMatchingSize(grad, b.partials());
    a.value() = new_value;
    return a;
  }

  // Rule #8: pow(-1, ±∞) returns 1.
  if (a.value() == -1 && std::isinf(b.value())) {
    // The resulting gradient is only well-defined when `a` had no gradient.
    const double grad = HasGrad(a) ? kNaN : 0.0;
    a.partials().SetConstantMatchingSize(grad, b.partials());
    a.value() = new_value;
    return a;
  }

  // Rule #4: pow(±0, -∞) returns +∞.
  // Rule #12: pow(base, -∞) returns +∞ for any |base|<1.
  // Rule #14: pow(base, +∞) returns +0 for any |base|<1.
  // Rule #13: pow(base, -∞) returns +0 for any |base|>1.
  // Rule #15: pow(base, +∞) returns +∞ for any |base|>1.
  // Note that Rule #8 above already covered when base == -1.
  // Note that Rule #9 above already covered when base == +1.
  if (std::isinf(b.value())) {
    // The resulting gradient is well-defined everywhere.
    const double grad = 0.0;
    a.partials().SetConstantMatchingSize(grad, b.partials());
    a.value() = new_value;
    return a;
  }

  // --- As of this point, both arguments are known to be finite. ---
  DRAKE_ASSERT(std::isfinite(a.value()));
  DRAKE_ASSERT(std::isfinite(b.value()));

  // Rule #11: pow(base, exp) returns NaN if base is finite and negative and
  // exp is finite and non-integer.
  if ((a.value() < 0) && !IsInteger(b.value())) {
    const double grad = kNaN;
    a.partials().SetConstantMatchingSize(grad, b.partials());
    a.value() = new_value;
    return a;
  }

  // Rule #1: pow(+0, exp), where exp is a negative odd integer, returns +∞.
  // Rule #2: pow(-0, exp), where exp is a negative odd integer, returns -∞.
  // Rule #3: pow(±0, exp), where exp is negative, finite, and is an even
  //   integer or a non-integer, returns +∞.
  // Rule #4: pow(±0, -∞) returns +∞.
  // Rule #5: pow(+0, exp), where exp is a positive odd integer, returns +0.
  // Rule #6: pow(-0, exp), where exp is a positive odd integer, returns -0.
  // Rule #7: pow(±0, exp), where exp is positive non-integer or a positive
  //   even integer, returns +0.
  if (a.value() == 0) {
    // The resulting gradient is ill-defined for #2 and #6.
    const double grad = std::signbit(new_value) ? kNaN : 0.0;
    a.partials().SetConstantMatchingSize(grad, b.partials());
    a.value() = new_value;
    return a;
  }

  // Phew! All of the IEEE special cases have been handled now.
  DRAKE_ASSERT(std::isfinite(new_value));

  // For the gradient, we have:
  //  ∂/∂x aᵇ = aᵇ⁻¹ (b a' + a ln(a) b')
  // In the particular case where `a < 0` and `b' == 0`, we need to be careful
  // not to introduce NaNs into the gradient via the second term via `ln(a)`.
  const double a_bn1 = std::pow(a.value(), b.value() - 1);
  const double a_scale = a_bn1 * b.value();
  a.partials().Mul(a_scale);
  if (HasGrad(b)) {
    const double b_scale = a_bn1 * a.value() * std::log(a.value());
    a.partials().AddScaled(b_scale, b.partials());
  }
  a.value() = new_value;
  return a;
}

AutoDiff pow(double a, const AutoDiff& b) {
  return pow(AutoDiff{a}, b);
}

AutoDiff pow(AutoDiff a, double b) {
  return pow(std::move(a), AutoDiff{b});
}

std::ostream& operator<<(std::ostream& s, const AutoDiff& x) {
  return s << fmt::format("{}", x.value());
}

}  // namespace ad
}  // namespace drake
